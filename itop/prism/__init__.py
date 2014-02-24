"""
A package for handeling prism testing.
"""

from math import sin, cos, acos, atan, degrees
from itop import N_HPFS, N_AIR

def deflection_angle(undeflected_beam, deflected_beam):
  """Return the angle of beam deflection in radians."""
  return acos(undeflected_beam.direction.dot(deflected_beam.direction).value)

def prism_angle(alpha, index_quartz=N_HPFS, index_air=N_AIR):
  """Return the prism angle given the beam's angle of deflection
  when the prism is moved into the beam path.

  """
  return atan(sin(alpha) / ((index_quartz / index_air) - cos(alpha)))

def angle_from_beams(undeflected_beam, deflected_beam, **kwargs):
  """Return the prism angle in degrees from a pair of deflected and
  undeflected beam trajectories.

  Any keyword arguments are passed to the deflection angle calculation
  itop.prism_angle().
  """
  return degrees(prism_angle(
      deflection_angle(undeflected_beam, deflected_beam), **kwargs))
