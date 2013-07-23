"""
A module for the mathematics of optics.

"""
from numpy import array, linalg, dot, sqrt
import math
from itop.math import Vector
from itop.math.linalg import normalize

def refract(ray, normal, origin_index, final_index):
  """Returns the normalized direction of a given ray (normalized or not) after
  refraction through a boundary between two media with given normal vector and
  indexes of refraction

  """
  original_direction = normalize(ray)
  normal = normalize(normal)
  index_ratio = origin_index / final_index
  incidence = -dot(original_direction, normal)
  complement = sqrt(1.0 - index_ratio**2 * (1.0 - incidence**2))
  sign = 1.0 if incidence > 0 else -1.0
  return (original_direction * index_ratio + normal *
      sign * (index_ratio * incidence - complement))

def reflection_normal(outgoing_ray, incoming_ray):
  """Returns the normal vector between incoming and outgoing
  reflection rays.

  """
  ray1 = -incoming_ray
  ray2 = outgoing_ray
  return normalize(ray1 + ray2)

def reconstruct_mirror_normal(
    outgoing_ray,
    incoming_ray=None,
    surface_normal=None,
    inside_index=None,
    outside_index=None):
  """Reconstructs the mirror normal vector given the incoming ray
  vector, outgoing ray vector, front face normal vector and the indexes of
  refraction.

  """
  if inside_index is None:
    inside_index = 1.4608
  if outside_index is None:
    outside_index = 1.000277
  if incoming_ray is None:
    incoming_ray = [0, 0, -1]
  if surface_normal is None:
    surface_normal = [0, 0, 1]
  outgoing_ray = normalize(outgoing_ray)
  incoming_ray = normalize(incoming_ray)
  surface_normal = normalize(surface_normal)
  inner_upstream = -refract(
      -outgoing_ray, surface_normal, outside_index, inside_index)
  inner_downstream = refract(
      incoming_ray, surface_normal, outside_index, inside_index)
  return reflection_normal(inner_upstream, inner_downstream)

def radius_from_normals(normal_1, normal_2, impact_1, impact_2):
  """Return the radius of the sphere described by the normals at the given
  (x,y) impact positions.

  This does not check nor take into account acoplanarity of the normals.
  """
  n1n2 = normal_1.dot(normal_2)
  x1x2 = impact_1[0] * impact_2[0]
  y1y2 = impact_1[1] * impact_2[1]
  r1r1 = impact_1.dot(impact_1)
  r2r2 = impact_2.dot(impact_2)
  a = (n1n2**2 - 1)
  b = (r2r2 + r1r1 - 2 * n1n2 * (x1x2 + y1y2))
  c = (x1x2 + y1y2)**2 - (r2r2 * r1r1)
  return sqrt((-b - sqrt(b**2 - 4 * a * c))/(2 * a))

def focus(beam_0, beam_1, plane='T'):
  """Returns the focal point for the given beams.

  The beams must be of the type itop.beam.Beam and fully initialized.

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'

  """
  index = 1 if plane is 'S' else 0
  # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
  coefficients = array(
      [[beam_0.direction[2], -beam_0.direction[index]],
       [beam_1.direction[2], -beam_1.direction[index]]])
  ordinates = array(
      [(beam_0.direction[2] * beam_0.intercept[index] -
        beam_0.direction[index] * beam_0.intercept[2]),
       (beam_1.direction[2] * beam_1.intercept[index] -
        beam_1.direction[index] * beam_1.intercept[2])])
  solution = linalg.solve(coefficients, ordinates)
  fraction_a = ((solution[1] - beam_0.intercept[2]) / beam_0.direction[2])
  fraction_b = ((solution[1] - beam_1.intercept[2]) / beam_1.direction[2])

  focus_0 = Vector(beam_0.intercept + fraction_a * beam_0.direction).array()
  focus_1 = Vector(beam_1.intercept + fraction_a * beam_1.direction).array()
  focus_0_error = [
      sqrt(sum((error**2 for error in axis)))
          for axis in zip(*(beam_0.position(ordinate, index).maximal_errors()
          for index, ordinate in enumerate(focus_0)))]
  focus_1_error = [
      sqrt(sum((error**2 for error in axis)))
          for axis in zip(*(beam_1.position(ordinate, index).maximal_errors()
          for index, ordinate in enumerate(focus_1)))]

  return [Vector(focus_0, [focus_0_error]), Vector(focus_1, [focus_1_error])]


