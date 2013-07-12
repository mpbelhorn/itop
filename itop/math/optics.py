"""
A module for the mathematics of optics.

"""
from numpy import array, linalg, dot
import math
from itop.math import Vector

def refract(ray, normal, origin_index, final_index):
  """Returns the normalized direction of a given ray (normalized or not) after
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

def reflection_normal(outgoing_ray, incoming_ray):
  """Returns the normal vector between incoming and outgoing
  reflection rays.

  """
  ray1 = -Vector(incoming_ray)
  ray2 = Vector(outgoing_ray)
  return (ray1 + ray2).normalize()

def reconstruct_mirror_normal(
    outgoing_ray,
    incoming_ray=Vector([0,0,-1], [[1.0e-5, 1.0e-5, 0]]),
    surface_normal=Vector([0,0,1], [[5.0e-4, 5.0e-4, 0]]),
    inside_index=1.4608,
    outside_index=1.000277):
  """Reconstructs the mirror normal vector given the incoming ray
  vector, outgoing ray vector, front face normal vector and the indexes of
  refraction.

  """
  outgoing_ray = Vector(outgoing_ray)
  incoming_ray = Vector(incoming_ray)
  surface_normal = Vector(surface_normal)
  inner_upstream = -refract(
      -outgoing_ray, surface_normal, outside_index, inside_index)
  inner_downstream = refract(
      incoming_ray, surface_normal, outside_index, inside_index)
  return reflection_normal(inner_upstream, inner_downstream)

def radius_from_normals(normal1, normal2, x_displacement, y_displacement):
  """Calculates the radius of curvature given two normal vectors and their
  relative displacements in the xy plane assuming reflection off a
  perfectly spherical surface.

  The displacements are given as (x, y) = r2 - r1 where the r are position
  vectors to the input positions in the plane perpendicular to the optical
  axis.

  """
  x_sum = normal1[0] + normal2[0]
  y_sum = normal1[1] + normal2[1]
  z_difference = normal1[2]**2 - normal2[2]**2
  return -(x_displacement * x_sum + y_displacement * y_sum) / z_difference

def focus(beam_a, beam_b, plane='T'):
  """Returns the focal point for the given beams.

  The beams must be of the type itop.beam.Beam and fully initialized.

  Takes the following optional keyword argument:
  plane  --  Selects the (T)angential or (S)agittal focal plane.
             Accpetable values are 'T' (default) or 'S'

  """
  index = 1 if plane is 'S' else 0
  beam_a.intercept = beam_a.intercept
  beam_b.intercept = beam_b.intercept
  # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
  coefficients = array(
      [[beam_a.slope[2], -beam_a.slope[index]],
       [beam_b.slope[2], -beam_b.slope[index]]])
  ordinates = array(
      [(beam_a.slope[2] * beam_a.intercept[index] -
        beam_a.slope[index] * beam_a.intercept[2]),
       (beam_b.slope[2] * beam_b.intercept[index] -
        beam_b.slope[index] * beam_b.intercept[2])])
  solution = linalg.solve(coefficients, ordinates)
  fraction_a = ((solution[1] - beam_a.intercept[2]) / beam_a.slope[2])
  fraction_b = ((solution[1] - beam_b.intercept[2]) / beam_b.slope[2])
  return [(beam_a.intercept + fraction_a * beam_a.slope),
          (beam_b.intercept + fraction_b * beam_b.slope)]


