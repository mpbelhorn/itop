"""
Functions for linear algebraic manipulation of vectors.
"""

import numpy as np

def rotationMatrix(theta, axis):
  """
  Returns the matrix describing the rotation about an axis '[x,y,z]'
  by an angle 'theta'.
  """
  axis = axis/np.sqrt(np.dot(axis, axis))
  a = np.cos(theta/2)
  b,c,d = axis * np.sin(theta/2)
  return np.array([[a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c)],
                   [2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b)],
                   [2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c]])

def rotateVector(vector, theta, axis):
  """
  Rotates a given vector by the angle theta about a given axis.
  """
  return np.dot(rotationMatrix(theta, axis), vector)

def normalize(vector):
  """
  Returns a normalized vector parallel to the given vector.
  """
  return vector / np.sqrt(np.dot(vector, vector))

