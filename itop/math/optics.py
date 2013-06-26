"""
A module for the mathematics of optics.
"""
from numpy import array, linalg, dot
import math

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


def focus(trajectory_a, trajectory_b, plane='T'):
  """
  Returns the focal point for the given beam trajectories.

  The beam trajectories are dictionaries of the form:
    {upstream xyz intercept,
     upstream xyz error,
     downstream xyz intercept,
     downstream xyz error,
     xyz slope}
  where the p_upstream and p_downstream are (x,y,z) points on the beam
  in the specified relative positions.

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  iua = trajectory_a['upstream point']
  iub = trajectory_b['upstream point']
  ida = trajectory_a['downstream point']
  idb = trajectory_b['downstream point']
  sa = ida - iua
  sb = idb - iub
  # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
  coefficients = array([[sa[2], -sa[index]], [sb[2], -sb[index]]])
  ordinates = array([sa[2] * iua[index] - sa[index] * iua[2],
                     sb[2] * iub[index] - sb[index] * iub[2]])
  solution = linalg.solve(coefficients, ordinates)
  fraction_a = ((solution[1] - iua[2]) / sa[2])
  fraction_b = ((solution[1] - iub[2]) / sb[2])
  focus_a = (iua + fraction_a * sa)
  focus_b = (iub + fraction_b * sb)
  return [focus_a, focus_b]


def focusWithUncertainty(trajectory_a, trajectory_b, plane='T'):
  """
  Returns the focal point and its uncertainty for the given beam trajectories.

  The beam intercepts are dictionaries of the same form as for itop.optics.focus().

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  foci = []
  signs_list = [[ 0, 0],  # Central Value
                [ 1,-1],  # Upstream Limit
                [-1, 1]]  # Downstream Limit
  iua = trajectory_a['upstream point']
  eua = trajectory_a['upstream error']
  iub = trajectory_b['upstream point']
  eub = trajectory_b['upstream error']
  ida = trajectory_a['downstream point']
  eda = trajectory_a['downstream error']
  idb = trajectory_b['downstream point']
  edb = trajectory_b['downstream error']

  for signs in signs_list:
    offset = array([0, 0, 0])
    ri_a = iua + signs[0] * (offset + array(eua[index]))
    rf_a = ida + signs[0] * (offset + array(eda[index]))
    ri_b = iub + signs[1] * (offset + array(eub[index]))
    rf_b = idb + signs[1] * (offset + array(edb[index]))
    foci.append(focus(
        {'upstream point': ri_a, 'downstream point': rf_a},
        {'upstream point': ri_b, 'downstream point': rf_b},
        plane=plane))
  # TODO - If stage can move to focal point, verify the position is correct.
  delta1_a = (foci[1][0] - foci[0][0]).tolist()
  delta2_a = (foci[2][0] - foci[0][0]).tolist()
  delta1_b = (foci[1][1] - foci[0][1]).tolist()
  delta2_b = (foci[2][1] - foci[0][1]).tolist()
  return [[foci[0][0].tolist(), [delta1_a, delta2_a]],
          [foci[0][1].tolist(), [delta1_b, delta2_b]]]

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

