"""
Functions for linear algebraic manipulation of vectors.

"""

import numpy as np

def rotation_matrix(theta, axis):
  """Applies the Euler-Rodrigues formula to return the rotation matrix
  for a rotation about axis by the angle theta.

  """
  axis = axis/np.sqrt(np.dot(axis, axis))
  a = np.cos(theta/2)
  b, c, d = axis * np.sin(theta/2)
  return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                   [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                   [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])

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
  return vector / np.sqrt(np.dot(vector, vector))

