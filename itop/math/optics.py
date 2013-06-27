"""
A module for the mathematics of optics.
"""
from numpy import array, linalg, dot
import math
from itop.beam.beam import TrajectoryData

def refract(ray, normal, origin_index, final_index):
  """
  Returns the normalized direction of a given ray (normalized or not) after
  refraction through a boundary between two media with given normal vector and
  indexes of refraction
  """
  original_direction = array(ray) / linalg.norm(ray)
  normal = array(normal) / linalg.norm(normal)
  index_ratio = origin_index / final_index
  incidence = dot(-original_direction, normal)
  complement = math.sqrt(1.0 - index_ratio**2 * (1.0 - incidence**2))
  sign = 1.0 if incidence > 0 else -1.0
  return (index_ratio * original_direction +
      sign * (index_ratio * incidence - complement) * normal)

def reconstructMirrorNormal(downstream_ray, **kwargs):
  """
  Reconstructs the mirror normal vector given the upstream ray
  (incoming to front face) beam propagation vector and the downstream
  ray (outgoing from front face).
  """
  upstream_ray = kwargs.pop('upstream_ray', [0, 0, -1])
  face_normal = kwargs.pop('face_normal', [0, 0, 1])
  index_outside = kwargs.pop('index_outside', 1.000277)
  index_inside = kwargs.pop('index_inside', 1.4608)
  inner_upstream = refract(
      array(upstream_ray), face_normal, index_outside, index_inside)
  inner_downstream = -refract(
      -array(downstream_ray), face_normal, index_outside, index_inside)
  return ((inner_downstream - inner_upstream) /
      (linalg.norm(inner_downstream - inner_upstream)))

def radiusFromNormals(
    beam_a_normal, beam_b_normal,
    x_displacement, y_displacement):
  """
  Calculates the radius of curvature given two normal vectors and their
  relative displacements in the xy plane.
  """
  x_sum = beam_a_normal[0] + beam_b_normal[0]
  y_sum = beam_a_normal[1] + beam_b_normal[1]
  z_difference = beam_a_normal[2]**2 - beam_b_normal[2]**2
  return -(x_displacement * x_sum + y_displacement * y_sum) / z_difference

def focus(trajectory_a, trajectory_b, plane='T'):
  """
  Returns the focal point for the given beam trajectories.

  The beam trajectories must be passed as named tuples of the form given
  by itop.beam.Beam.dump(serializable=False).

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  aup = trajectory_a.upstream_point
  bup = trajectory_b.upstream_point
  adp = trajectory_a.downstream_point
  bdp = trajectory_b.downstream_point
  sa = adp - aup
  sb = bdp - bup
  # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
  coefficients = array([[sa[2], -sa[index]], [sb[2], -sb[index]]])
  ordinates = array([sa[2] * aup[index] - sa[index] * aup[2],
                     sb[2] * bup[index] - sb[index] * bup[2]])
  solution = linalg.solve(coefficients, ordinates)
  fraction_a = ((solution[1] - aup[2]) / sa[2])
  fraction_b = ((solution[1] - bup[2]) / sb[2])
  focus_a = (aup + fraction_a * sa)
  focus_b = (bup + fraction_b * sb)
  return [focus_a, focus_b]


def focusWithUncertainty(trajectory_a, trajectory_b, plane='T'):
  """
  Returns the focal point and its uncertainty for the given beam trajectories.

  The beam trajectories must be passed as named tuples of the same form as for
  itop.optics.focus().

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  foci = []
  signs_list = [[ 0, 0],  # Central Value
                [ 1,-1],  # Upstream Limit
                [-1, 1]]  # Downstream Limit
  aup = trajectory_a.upstream_point
  aue = trajectory_a.upstream_error
  bup = trajectory_b.upstream_point
  bue = trajectory_b.upstream_error
  adp = trajectory_a.downstream_point
  ade = trajectory_a.downstream_error
  bdp = trajectory_b.downstream_point
  bde = trajectory_b.downstream_error

  for signs in signs_list:
    offset = array([0, 0, 0])
    ri_a = aup + signs[0] * (offset + aue[index])
    rf_a = adp + signs[0] * (offset + ade[index])
    ri_b = bup + signs[1] * (offset + bue[index])
    rf_b = bdp + signs[1] * (offset + bde[index])
    foci.append(focus(
        TrajectoryData(None, ri_a, None, rf_a, None),
        TrajectoryData(None, ri_b, None, rf_b, None),
        plane=plane))
  # TODO - If stage can move to focal point, verify the position is correct.
  delta1_a = (foci[1][0] - foci[0][0])
  delta2_a = (foci[2][0] - foci[0][0])
  delta1_b = (foci[1][1] - foci[0][1])
  delta2_b = (foci[2][1] - foci[0][1])
  return [[foci[0][0], (delta1_a, delta2_a)],
          [foci[0][1], (delta1_b, delta2_b)]]

