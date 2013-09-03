"""
A module for the mathematics of optics.

"""
import numpy as np
from numpy import array, linalg, dot, sqrt
from itop.math import Vector
from itop.math.linalg import normalize

def refract(ray, normal, origin_index, final_index):
  """Returns the normalized direction of a given ray (normalized or not) after
  refraction through a boundary between two media with given normal vector and
  indexes of refraction

  """
  rho = final_index / origin_index
  ray_direction = normalize(ray)
  normal = normalize(normal)
  if normal.dot(ray_direction) > 0:
    normal = -normal
  incidence = dot(-ray_direction, normal)
  complement = sqrt(1.0 - (1.0 - incidence**2) / rho**2)
  return (ray_direction / rho + (incidence / rho - complement) * normal)

def reflection_normal(outgoing_ray, incoming_ray):
  """Returns the normal vector between incoming and outgoing
  reflection rays.

  """
  ray1 = normalize(-incoming_ray)
  ray2 = normalize(outgoing_ray)
  return normalize((ray1 + ray2)/2)

def reconstruct_mirror_normal(
    outgoing_ray, incoming_ray, surface_normal, inside_index, outside_index):
  """Reconstructs the mirror normal vector given the incoming ray
  vector, outgoing ray vector, front face normal vector and the indexes of
  refraction.

  """
  outgoing_ray = normalize(outgoing_ray)
  incoming_ray = normalize(incoming_ray)
  surface_normal = normalize(surface_normal)
  inner_downstream = -refract(
      -outgoing_ray, surface_normal, outside_index, inside_index)
  inner_upstream = refract(
      incoming_ray, surface_normal, outside_index, inside_index)
  return reflection_normal(inner_downstream, inner_upstream)

def radius_from_normals_old(normal_1, normal_2, impact_1, impact_2):
  """Return the radius of the sphere described by the normals at the given
  (x,y) impact positions.

  This does not check nor take into account acoplanarity of the normals.
  """

  n1n2 = normal_1.dot(normal_2)
  x1x2 = impact_1[0] * impact_2[0]
  y1y2 = impact_1[1] * impact_2[1]
  r1r1 = impact_1.dot(impact_1)
  r2r2 = impact_2.dot(impact_2)
  quad_a = (n1n2**2 - 1)
  quad_b = (r2r2 + r1r1 - 2 * n1n2 * (x1x2 + y1y2))
  quad_c = (x1x2 + y1y2)**2 - (r2r2 * r1r1)
  return sqrt((-quad_b - sqrt(quad_b**2 - 4 * quad_a * quad_c))/(2 * quad_a))

def radius_from_normals(normal_1, normal_2, input_1, impact_1, impact_2):
  alpha = abs(np.arccos(normal_1.dot(normal_2)))
  plane = normalize(np.cross(normal_1, normal_2))
  input_in_plane = normalize(input_1 - (input_1.dot(plane) * plane))
  theta = abs(np.arccos(normal_1.dot(input_in_plane)))
  separation = impact_1 - impact_2
  separation = np.append(separation, 0)
  separation -= separation.dot(plane) * plane
  separation = np.linalg.norm(separation)

  return separation / (2 * abs(np.sin(alpha/2)) * abs(np.cos(theta + alpha/2)))

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
  focus_1 = Vector(beam_1.intercept + fraction_b * beam_1.direction).array()
  focus_0_error = [
      sqrt(sum((error**2 for error in axis)))
          for axis in zip(*(beam_0.position(ordinate, index).maximal_errors()
          for index, ordinate in enumerate(focus_0)))]
  focus_1_error = [
      sqrt(sum((error**2 for error in axis)))
          for axis in zip(*(beam_1.position(ordinate, index).maximal_errors()
          for index, ordinate in enumerate(focus_1)))]

  return [Vector(focus_0, [focus_0_error]), Vector(focus_1, [focus_1_error])]


