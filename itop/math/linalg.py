"""
Functions for linear algebraic manipulation of vectors.

"""

import numpy as np
from numpy import cross, dot, array
from numpy.linalg import norm
from itop.math.measurements import Vector

def rotation_matrix(theta, axis):
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

def rotation_matrix_from_vectors(in1, in2=[0, 0, -1]):
  axis = [1, 0, 0]
  x1 = array(in1) / norm(in1)
  x2 = array(in2) / norm(in2)
  if all(x1 == x2):
    return identity(3)
  elif (all(cross(x1, axis) == np.zeros(3)) or
        all(cross(x2, axis) == np.zeros(3))):
    axis = cross(x1, x2) / norm(cross(x1, x2))
  y1 = cross(x1, axis) / norm(cross(x1, axis))
  y2 = cross(x2, axis) / norm(cross(x2, axis))
  z1 = cross(x1, y1) / norm(cross(x1, y1))
  z2 = cross(x2, y2) / norm(cross(x2, y2))
  return dot(array([x2, y2, z2]).T, np.linalg.inv([x1, y1, z1]).T)

def crazy_test(in1, in2=[0, 0, -1]):
  axis = Vector([1, 0, 0])
  x1 = Vector(in1).normalize()
  x2 = Vector(in2).normalize()
  if all(x1.array() == x2.array()):
    return np.identity(3)
  elif (all(cross(x1.array(), axis.array()) == np.zeros(3)) or
        all(cross(x2.array(), axis.array()) == np.zeros(3))):
    axis = Vector(cross(x1, x2)).normalize()
  y1 = Vector(cross(x1, axis)).normalize()
  y2 = Vector(cross(x2, axis)).normalize()
  z1 = Vector(cross(x1, y1)).normalize()
  z2 = Vector(cross(x2, y2)).normalize()
  return dot(array([x2, y2, z2]).T, np.linalg.inv(array([x1, y1, z1])).T)

def rotate_vector(vector, theta, axis):
  """Rotates a given vector by the angle theta about a given axis.

  """
  return np.dot(rotation_matrix(theta, axis), vector)

def rotate_yxz_tait_bryan(vector, angles):
  """Returns the vector rotated under the yx'z'' Tait-Bryan convention Euler
  angles. Angles must be an interable over (phi, theta, psi)

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

