"""
A package for handeling prism testing.
"""

from math import sin, cos, atan

def angle(alpha, index_quartz=1.4608, index_air=1.000277):
  """Return the prism angle given the beam's angle of deflection
  when the prism is moved into the beam path.

  """
  return atan(sin(alpha) / ((index_quartz / index_air) - cos(alpha)))


