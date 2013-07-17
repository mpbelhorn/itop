"""
Functions for linear algebraic manipulation of vectors.

"""

import numpy as np
from numpy import cross, dot, array
from numpy.linalg import norm
from itop.math.measurements import Vector, Value

def rotation_matrix_euler(theta, axis):
  """Applies the Euler-Rodrigues formula to return the rotation matrix
  for a rotation about axis by the angle theta.
  """
  axis = normalize(axis)
  erpa = np.cos(theta/2)
  erpb, erpc, erpd = axis * np.sin(theta/2)
  return np.array(
      [[erpa*erpa+erpb*erpb-erpc*erpc-erpd*erpd,
        2*(erpb*erpc-erpa*erpd),
        2*(erpb*erpd+erpa*erpc)],
       [2*(erpb*erpc+erpa*erpd),
        erpa*erpa+erpc*erpc-erpb*erpb-erpd*erpd,
        2*(erpc*erpd-erpa*erpb)],
       [2*(erpb*erpd-erpa*erpc),
        2*(erpc*erpd+erpa*erpb),
        erpa*erpa+erpd*erpd-erpb*erpb-erpc*erpc]])

def rotation_matrix_arrays(original, final=None):
  """Build a matrix T that transforms numpy.array original into final via
  T.dot(original) = final.

  This function attempts to use numpy functions that cannot process Value and
  Vector objects safely or reliably. Use transform_matrix() instead.
  """
  if final is None:
    final = [0, 0, -1]
  axis = [1, 0, 0]
  pp_1 = normalize(original)
  pp_2 = normalize(final)
  if all(pp_1 == pp_2):
    return np.identity(3)
  elif (all(cross(pp_1, axis) == np.zeros(3)) or
        all(cross(pp_2, axis) == np.zeros(3))):
    axis = cross(pp_1, pp_2) / norm(cross(pp_1, pp_2))
  op_1 = normalize(cross(pp_1, axis))
  op_2 = normalize(cross(pp_2, axis))
  ob_1 = normalize(cross(pp_1, op_1))
  ob_2 = normalize(cross(pp_2, op_2))
  return dot(
      array([pp_2, op_2, ob_2]).T,
      np.linalg.inv([pp_1, op_1, ob_1]).T)

def rotation_matrix(original, final=None):
  """Build a matrix T that transforms vector-like original into final via
  T.dot(original) = final.

  Output is in the form of a 3x3 numpy.ndarray of Value objects. The numpy
  dot function must be used to do subsequent matrix multiplication. The
  implementations of numpy.dot() and Value() allow for the propogation of
  error through matrix operations provided a numpy array is the left-hand
  operator.

  Input arguments are converted to Vector objects. Any error is propagated
  through to the matrix elements. When used to execute a transformation,
  the error on the output vector is propagated through.

  The algorithm
  =============
  For each input vector, form normalized vectors:
    pp = parallel to to the input,
    op = orthogonal to pp, and
    ob = orthogonal to both op and pp.
  As rows of a matrix, pp produces x coordinates in rank-reducing operations,
  op -> y, and ob -> z. The vector op can, in gengeral, be any arbitrary
  vector perpendicular to 'pp'. So that any polarization relative to the table
  is maintained through transformations, the beam x-axis is kept in the plane
  of the table by choosing 'op' to be orthogonal to the beam and the lab
  x-axis. The full transformation matrix is the one that moves
     [pp_1]   [pp_2]               [pp_2] ([pp_1])^(-1)
  T. [op_1] = [op_2] such that T = [op_2].([op_1])
     [ob_1]   [ob_2]               [ob_2] ([ob_1])
  If for some reason either pp is parallel to the lab x-axis, a fail safe
  choice of axis = pp_1 X pp_2 is used, but polarization information is
  silently lost. If pp_1 is parallel to pp_2, the 3x3 identity matrix is
  returned.
  """
  if final is None:
    final = [0, 0, -1]
  axis = Vector([1, 0, 0])
  pp_1 = Vector(original).normalize()
  pp_2 = Vector(final).normalize()
  if all(pp_1.array() == pp_2.array()):
    return array([[Value(1), Value(0), Value(0)],
                  [Value(0), Value(1), Value(0)],
                  [Value(0), Value(0), Value(1)]])
  elif (all(cross(pp_1.array(), axis.array()) == np.zeros(3)) or
        all(cross(pp_2.array(), axis.array()) == np.zeros(3))):
    axis = Vector(cross(pp_1, pp_2)).normalize()
  op_1 = Vector(cross(pp_1, axis)).normalize()
  op_2 = Vector(cross(pp_2, axis)).normalize()
  ob_1 = Vector(cross(pp_1, op_1)).normalize()
  ob_2 = Vector(cross(pp_2, op_2)).normalize()
  return dot(
      array([pp_2, op_2, ob_2]).T,
      np.linalg.inv(array([pp_1, op_1, ob_1])).T)

def rotate_vector(vector, theta, axis):
  """Rotates a given vector by the angle theta about a given axis.
  """
  return np.dot(rotation_matrix_euler(theta, axis), vector)

def rotate_yxz_tait_bryan(vector, angles):
  """Returns the vector rotated under the yx'z'' Tait-Bryan convention Euler
  angles. Angles must be an interable over (phi, theta, psi).
  """
  axes = [[0, 1, 0],
          [1, 0, 0],
          [0, 0, 1]]
  output = vector
  for axis, angle in enumerate(angles):
    output = rotate_vector(output, angle, axes[axis])
  return output

def normalize(vector):
  """Returns a normalized vector parallel to the given vector.
  """
  return np.array(vector) / np.linalg.norm(np.array(vector))

