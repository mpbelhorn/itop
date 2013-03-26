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


def focus(intercepts_a, intercepts_b, plane='T'):
  """
  Returns the focal point for the given beam intercepts.

  The beam intercepts are lists of the form (p_upstream, p_downstream)
  where the p_upstream and p_downstream are (x,y,z) points on the beam
  in the specified relative positions.

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  sa = intercepts_a[1] - intercepts_a[0]
  sb = intercepts_b[1] - intercepts_b[0]
  # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
  coefficients = array([[sa[2], -sa[index]], [sb[2], -sb[index]]])
  ordinates = array([sa[2] * intercepts_a[index] - sa[index] * intercepts_a[2],
                     sb[2] * intercepts_b[index] - sb[index] * intercepts_b[2]])
  solution = linalg.solve(coefficients, ordinates)
  fraction_a = ((solution[1] - intercepts_a[2]) / sa[2])
  fraction_b = ((solution[1] - intercepts_b[2]) / sb[2])
  focus_a = (intercepts_a + fraction_a * sa)
  focus_b = (intercepts_b + fraction_b * sb)
  return (focus_a, focus_b)


def focusWithUncertainty(intercepts_a, uncertainties_a,
    intercepts_b, uncertainties_b, plane='T'):
  """
  Returns the focal point and its uncertainty for the given beam intercepts.

  The beam intercepts are lists of the form (p_upstream, p_downstream)
  where the p_upstream and p_downstream are (x,y,z) points on the beam
  in the specified relative positions.

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'
  """
  index = 1 if plane is 'S' else 0
  foci = []
  signs_list = [[ 0, 0],
                [ 1,-1],
                [-1, 1]]
  for signs in signs_list:
    offset = array([0, 0, 0])
    ri_a = intercepts_a[0] + signs[0] * (
        offset + array(uncertainties_a[0][index]))
    rf_a = intercepts_a[1] + signs[0] * (
          offset + array(uncertainties_a[1][index]))
    ri_b = intercepts_b[0] + signs[1] * (
        offset + array(uncertainties_b[0][index]))
    rf_b = intercepts_b[1] + signs[1] * (
        offset + array(uncertainties_b[1][index]))
    foci.append(focus([ri_a, rf_a], [ri_b, rf_b], plane=plane))
  # TODO - If stage can move to focal point, verify the position is correct.
  return [foci[0], [foci[1], foci[2]]]

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

