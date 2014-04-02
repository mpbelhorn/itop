"""A collection of raytracing tools to simulate itop mirror testing."""
import abc
import numpy as np
from math import sqrt
from numpy import array, dot
from itop import N_HPFS, N_AIR
from itop.math.linalg import normalize, rotation_matrix_arrays
from itop.math.linalg import rotation_matrix_euler as rotation_matrix
from itop.beam.alignment import Alignment as DataAlignment
from itop.beam.beam import Beam as DataBeam
from itop.beam.datapoint import DataPoint

class Ray(object):
  """A beam segment connecting two optical elements."""
  def __init__(self, direction, position):
    """Construct a Ray from a direction and a position from the origin.
    The direction will be forced normal."""
    self.direction = normalize(direction)
    self.position = np.array(position)

  def __repr__(self):
    return 'ray({}, {})'.format(self.direction, self.position)

  def propagate(self, time):
    """Return the position of the ray at some parametric 'time' later."""
    return Ray(self.direction, self.position + time * self.direction)

  def sample(self, z_position):
    """Return the beam position at the given z-coordinate."""
    if self.direction[2] != 0:
      z_plane = PlaneSurface([0, 0, 1], [0, 0, z_position])
      if (z_plane.position() - self.position).dot(self.direction) > 0:
        ray = self
      else:
        ray = Ray(-self.direction, self.position)
    return ray.propagate(z_plane.time_to_bound(ray)).position


class RayTraceError(Exception):
  """An exception to handle ray tracing errors."""
  pass

class _Surface(object):
  """Base class for optical surfaces.

  In order to define closed volumes, the normal vectors for all surfaces must
  face the outside of the object. When reflecting or refracting, the code
  assumes the user is calling the operation on a valid interaction and
  flips the normal such that the dot product of the ray and the normal is less
  than zero.
  """
  __metaclass__ = abc.ABCMeta

  def __init__(self, name=None, reflective=False):
    self._name = name
    self._reflective = reflective

  def __repr__(self):
    return 'boundary({})'.format(self._name)

  def is_reflective(self):
    """Return True if the surface is reflective. Otherwise, return False."""
    return self._reflective

  @abc.abstractmethod
  def translate(self, displacement):
    """Move the object by displacement vector but maintain orientation."""
    print 'Invoked abstract {}.translate({})'.format(self, displacement)
    return

  @abc.abstractmethod
  def _rotate_about_origin(self, angle, axis):
    """Rotate the object about an axis at the origin by an angle."""
    print 'Invoked abstract {}._rotate_about_origin({}, {})'.format(
        self, angle, axis)
    return

  @abc.abstractmethod
  def normal(self, position):
    """Return the exterior normal vector at a position on the surface."""
    print 'Invoked abstract {}.normal({})'.format(self, position)
    return None

  @abc.abstractmethod
  def time_to_bound(self, ray):
    """Return the parametric 'time' to ray intersection with the surface."""
    print 'Invoked abstract {}.time_to_bound({})'.format(self, ray)
    return None

  @abc.abstractmethod
  def contains(self, position):
    """Return True if position is on interior side of the surface."""
    print 'Invoked abstract {}.contains({})'.format(self, position)
    return False

  def rotate(self, angle, axis, position=None):
    """Rotate the surface by an angle about an axis at a position.
    If no position is given, surface is rotated about the origin."""
    if position is not None:
      pos = np.array(position)
      self.translate(-pos)
      self._rotate_about_origin(angle, axis)
      self.translate(pos)
    else:
      self._rotate_about_origin(angle, axis)

  def refract(self, ray, rho):
    """Return a ray propagated through refraction given the
    normal vector and the ratio of indexes rho=n2/n1."""
    normal = self.normal(ray.position)
    if normal.dot(ray.direction) > 0:
      normal = -normal
    incidence = dot(-ray.direction, normal)
    complement = sqrt(1.0 - (1.0 - incidence**2) / rho**2)
    return Ray((ray.direction / rho +
        (incidence / rho - complement) * normal), ray.position)

  def reflect(self, ray):
    """Return the direction of propagation through reflection."""
    normal = self.normal(ray.position)
    if normal.dot(ray.direction) > 0:
      normal = -normal
    return Ray(
        ray.direction - 2 * dot(ray.direction, normal) * normal, ray.position)

  def propagate(self, ray, index_0, index_1):
    """Propagate the beam from region of refractive index index_0
    into a region of refractive index index_1."""
    if self._reflective:
      return self.reflect(ray)
    else:
      return self.refract(ray, index_1/index_0)



class PlaneSurface(_Surface):
  """A planar surface."""
  def __init__(self, normal, position, name=None, reflective=False):
    """Construct a plane from a normal vector and the
    position to a point on the plane."""
    _Surface.__init__(self, name, reflective)
    self._normal = normalize(normal)
    self._position = np.array(position)

  def translate(self, displacement):
    """Translate the surface by the displacement with fixed orientation."""
    self._position = self._position + np.array(displacement)

  def _rotate_about_origin(self, angle, axis):
    """Rotate the surface by angle about an axis at the origin."""
    matrix = rotation_matrix(angle, axis)
    self._normal = matrix.dot(self._normal)
    self._position = matrix.dot(self._position)

  def time_to_bound(self, ray):
    """Return the time-to-intersect if an intersection occurs, or None."""
    incidence_cosine = dot(self._normal, ray.direction)
    aplanarity = (self._position - ray.position).dot(self._normal)
    if incidence_cosine == 0:
      # Ray parallel to plane. If aplanarity also 0, ray is in the plane.
      return None
    time = aplanarity / incidence_cosine
    return time if time > 0 else None

  def normal(self, position):
    """Return the normal vector."""
    return self._normal

  def position(self):
    """Return the position vector to the point on the surface closest to
    the origin."""
    return self._position

  def contains(self, position):
    """Return true if the position is below the surface w.r.t. the normal."""
    return (position - self._position).dot(self.normal(position)) < 0


class SphericalSurface(_Surface):
  """A spherical surface."""
  def __init__(self, center, radius, name=None, reflective=True):
    _Surface.__init__(self, name, reflective)
    self._center = np.array(center)
    self._radius = radius
    self._position = self._center

  def contains(self, position):
    """Return true if the position is inside the volume of the sphere."""
    return np.linalg.norm(position - self._center) < self._radius

  def translate(self, displacement):
    """Translate the surface by the displacement with fixed orientation."""
    self._center = self._center + np.array(displacement)
    self._position = self._position + np.array(displacement)

  def _rotate_about_origin(self, angle, axis):
    """Rotate the surface by angle about an axis at the origin."""
    matrix = rotation_matrix(angle, axis)
    self._center = matrix.dot(self._center)

  def normal(self, point):
    """Return the normal at the point on the surface."""
    point = self._center - np.array(point)
    # if abs(point.dot(point) - self._radius**2) > 1e-15:
    #   raise RayTraceError(
    #       'Cannot compute normal. Point is too far from surface ({}).'.format(
    #       (abs(point.dot(point) - self._radius**2))))
    return normalize(point / self._radius)

  def time_to_bound(self, ray):
    """Return the time-to-intersect if an intersection occurs, or None."""
    qr_a = ray.direction.dot(ray.direction)
    qr_b = 2 * (ray.position - self._center).dot(ray.direction)
    qr_c = np.linalg.norm(ray.position - self._center)**2 - self._radius**2
    radical_arg = qr_b**2 - (4 * qr_a * qr_c)
    try:
      if qr_b < 0:
        quadratic = (-qr_b - sqrt(radical_arg)) / 2
      else:
        quadratic = (-qr_b + sqrt(radical_arg)) / 2
      if quadratic != 0.0:
        return min((i for i in (quadratic / qr_a, qr_c / quadratic) if i > 0))
    except ValueError:
      # Root is imaginary. Pass this to return None.
      pass
    return None


class _OpticalElement(object):
  """Base class for optical elements."""
  def __init__(self, name=None, index=N_AIR):
    self._name = name
    self.index = index
    self._bounds = []

  def add_boundary(self, boundary):
    """Add a boundary to the element."""
    self._bounds.append(boundary)

  def next_hit(self, ray):
    """Return the time-till-boundary and boundary of the next intersection."""
    hit_candidates = [(i.time_to_bound(ray), i) for i in self._bounds]
    try:
      # WARNING - A hard cut on 'times' smaller than 10^-9 is made to exclude
      # a beam reinteracting with the same barrier. This cuts out any legitimate
      # interactions closer than 1nm of the beam position.
      return (sorted([(time, surface) for time, surface in hit_candidates
          if time is not None and time > 1e-9 and all(
              [b.contains(ray.propagate(time).position) for b in self._bounds
                  if b is not surface])])[0])
    except IndexError:
      return None

  def translate(self, displacement):
    """Move the optical element by the displacement with fixed orientation."""
    for bound in self._bounds:
      bound.translate(displacement)

  def rotate(self, angle, axis, position=None):
    """Rotate the element about the axis located at the position by the angle.
    If no position is given, the element is rotated about the origin."""
    for bound in self._bounds:
      bound.rotate(angle, axis, position)


class Air(_OpticalElement):
  """The labratory air."""
  def __init__(self):
    _OpticalElement.__init__(self, 'air', N_AIR)

class ItopMirror(_OpticalElement):
  """A model of a production itop mirror."""
  def __init__(self, radius, dimensions, index=N_HPFS):
    """Construct an itop mirror. Be default, the mirror is located with front
    face in the z=0 plane and substrate centered on the z-zxis facing z+."""
    _OpticalElement.__init__(self, 'itop mirror', index)
    self.add_boundary(SphericalSurface(
        [0, 0, radius - dimensions[2]], radius, name='-z',
        reflective=True))
    self.add_boundary(PlaneSurface(
        [0, 0, 1], [0, 0, 0], name='+z'))
    self.add_boundary(PlaneSurface(
        [0, 1, 0], [0, dimensions[1]/2, 0], name='+y'))
    self.add_boundary(PlaneSurface(
        [0, -1, 0], [0, -dimensions[1]/2, 0], name='-y'))
    self.add_boundary(PlaneSurface(
        [1, 0, 0], [dimensions[0]/2, 0, 0], name='+x'))
    self.add_boundary(PlaneSurface(
        [-1, 0, 0], [-dimensions[0]/2, 0, 0], name='-x'))


class Beam(object):
  """A collection of ray segments joined by their interactions with optical
  elements."""

  def __init__(self, initial_direction, initial_position,
      initial_element=None, jitter=None):
    """Create a beam along the initial direction and position given. Optionally,
    jitter can be added to the beam by passing the standard deviation of the
    polar direction in radians."""
    if jitter is not None:
      initial_direction = normalize(initial_direction)
      initial_direction += np.linalg.inv(
          rotation_matrix_arrays(initial_direction)).dot(array(
              [np.random.normal(0, jitter), np.random.normal(0, jitter), 0]))
    self._ray = Ray(initial_direction, initial_position)
    self._history = [(0, self._ray)]
    self._element = Air() if initial_element is None else initial_element

  def _next_hit(self, elements):
    """Find the next interaction with the set of optical elements."""
    hits = ((item, item.next_hit(self._ray)) for item in elements)
    try:
      return sorted(
          [(hit[0], hit[1], item) for item, hit in hits if hit is not None])[0]
    except IndexError:
      return None

  def ray(self):
    """Return the beam segment ray."""
    return self._ray

  def propagate(self, elements):
    """Propagate a beam through the simulation over all interactions with
    elements and boundaries.
    """
    while True:
      try:
        tti, bound, next_element = self._next_hit(elements)
      except TypeError:
        # That's all, folks!
        return
      self._ray = self._ray.propagate(tti)
      self._history.append((self._history[-1][0] + tti, self._ray))
      if not bound.is_reflective() and self._ray.direction.dot(
          bound.normal(self._ray.position)) > 0:
        next_element = Air()
      self._ray = bound.propagate(
          self._ray, self._element.index, next_element.index)
      self._history.append((self._history[-1][0] + tti, self._ray))
      self._element = next_element

def simulate_alignment(
    beam_a_direction, beam_b_direction, beam_a_intercept, separation,
    samples=25, calibration_file=None):
  """Return the beam construction parameters and an itop.Alignment of the
  simulated beams."""
  alignment = DataAlignment(calibration_file)
  ray_a = Ray(beam_a_direction, beam_a_intercept)
  ray_a = Ray(beam_a_direction, ray_a.sample(150))

  ray_b = Ray(beam_b_direction,
      np.array(beam_a_intercept) + np.array(separation))
  ray_b = Ray(beam_b_direction, ray_b.sample(150))
  beam_a = DataBeam()
  beam_b = DataBeam()

  for tracker_z in np.arange(125, -95, -220/samples).tolist() + [-95]:
    beam_a.add_sample(ray_a.sample(tracker_z))
    beam_b.add_sample(ray_b.sample(tracker_z))
  alignment.beams = [beam_a, beam_b]
  alignment.displacements = [
      alignment.beams[index].intercept - alignment.beams[0].intercept
      for index in range(2)]
  return alignment


def simulate_data(
    start, stop, step, mirror_height, calibration,
    mirror_parameters=None, beam_a_parameters=None, beam_b_parameters=None):
  """Return a sample of simulated raw data on the half-open interval
  [start, stop) of mirror stage positions. Each sample is spaced by
  'step' mm. The mirror is positioned at a height 'mirror_height' with respect
  to the mirror calibration point. The mirror calibration must be specified to
  correctly position the mirror with respect to the tracker.

  By default, this function simulates perfectly parallel beams, separated by
  [-50, -5, 0] with no jitter impinging upon a R=7000mm itop mirror on a stage
  with no wobble.

  The mirror and beam parameters can be substituted by passing dictionaries to
  the appropriate keyword arguments. The content of the dictionaries for
  mirror and beam parameters must reflect the required and optional arguments
  to raytrace.ItopMirror() and raytrace.Beam() respectively. See those objects
  for available parameters.

  Beam parameters are expressed in the tracker coordinate frame.
  """
  if mirror_parameters is None:
    mirror_parameters = {
        'radius':7000,
        'dimensions':[440, 20, 20],
        }
  if beam_a_parameters is None:
    beam_a_parameters = {
        'initial_direction': [0, 0, -1],
        'initial_position': [122, 0, 150]
        }
  if beam_b_parameters is None:
    beam_b_parameters = {
        'initial_direction': [0, 0, -1],
        'initial_position': [72, -5, 150],
        }

  data = []
  mirror = ItopMirror(**mirror_parameters)
  # Move the mirror to the mirror calibration point in the tracker
  # frame (tracker_cal - mirror_cal)
  mirror.translate(array(calibration))
  mirror.translate([start - step, mirror_height, 0])
  for mirror_stage_position in np.arange(start, stop, step):
    # For a given mirror position, the mirror stage is translated in the
    # opposite direction.
    mirror.translate([step, 0, 0])
    simulated_beam_a = Beam(**beam_a_parameters)
    simulated_beam_a.propagate([mirror])
    simulated_beam_b = Beam(**beam_b_parameters)
    simulated_beam_b.propagate([mirror])
    beam_a = DataBeam()
    beam_b = DataBeam()
    for z_coordinate in np.arange(-95, 125, 220/5).tolist() + [125]:
      beam_a.add_sample(simulated_beam_a.ray().sample(z_coordinate))
      beam_b.add_sample(simulated_beam_b.ray().sample(z_coordinate))
    data.append(
        DataPoint(
            [mirror_stage_position, mirror_height, 0],
            [beam_a, beam_b]))
  return data
