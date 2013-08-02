"""A collection of raytracing tools to simulate itop mirror testing."""
import abc
import numpy as np
from math import sqrt
from numpy import array, dot
from itop.math.linalg import normalize, rotation_matrix_arrays
from itop.math.linalg import rotation_matrix_euler as rotation_matrix

def null_wobble(stage_position):
  """Return 0 for all input."""
  return 0

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

  @abc.abstractmethod
  def translate(self, displacement):
    """Move the object by displacement vector but maintain orientation."""
    print 'Invoked abstract {}.translate({})'.format(self, displacement)
    return

  @abc.abstractmethod
  def _rotate_about_origin(self, angle, axis):
    """Rotate the object about an axis at the origin by an angle."""
    print 'Invoked abstract {}._rotate_about_origin({}, {})'.format(self, angle, axis)
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
    """Rotate the surface by angle radians about an axis passing through a
    given position. If no position is given, surface is rotated about the origin."""
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
    complement = sqrt(1.0 - rho**2 * (1.0 - incidence**2))
    sign = 1.0 if incidence > 0 else -1.0
    return Ray(ray.direction / rho + sign * (
        (incidence / rho - complement) * normal), ray.position)

  def reflect(self, ray):
    """Return the direction of propagation through reflection."""
    normal = self.normal(ray.position)
    if normal.dot(ray.direction) > 0:
      normal = -normal
    return Ray(
        ray.direction - 2 * dot(ray.direction, normal) * normal, ray.position)

  def propagate(self, ray, index_0, index_1):
    """Propagate the beam given the surface from a region of refractive index index_0
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

  def contains(self, position):
    """Return true if the position is below the surface w.r.t. the normal."""
    return ((position - self._position).dot(self.normal(position)) < 0)


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

  def _rotate_about_origin(self, angle, axis):
    """Rotate the surface by angle about an axis at the origin."""
    matrix = rotation_matrix(angle, axis)
    self._center = matrix.dot(self._center)

  def normal(self, point):
    """Return the normal at the point on the surface."""
    point = np.array(point) - self._center
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
  def __init__(self, name=None, index=1.000277):
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
      # FIXME - A hard cut on 'times' smaller than 10^-9 is made to exclude
      # a beam reinteracting with the same barrier. This cuts out any legitimate
      # interactions closer than 1nm of the beam position.
      return (sorted([(time, surface) for time, surface in hit_candidates
          if time is not None and time > 1e-9 and all(
              [b.contains(ray.propagate(time).position) for b in self._bounds
                  if b is not surface])])[0])
    except IndexError:
      return None

class Air(_OpticalElement):
  """The labratory air."""
  def __init__(self):
    _OpticalElement.__init__(self, 'air', 1.000277)

class ItopMirror(_OpticalElement):
  def __init__(self, radius, dimensions):
    """Construct an itop mirror. Be default, the mirror is located with front
    face in the z=0 plane and substrate centered on the z-zxis facing z+."""
    _OpticalElement.__init__(self, 'itop mirror', 1.47)
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
  def __init__(self, direction, initial_position, jitter=None):
    """Create a beam along the initial direction and position given. Optionally,
    jitter can be added to the beam by passing the standard deviation of the
    polar direction in radians."""
    if jitter is not None:
      direction = normalize(direction)
      direction += np.linalg.inv(
          rotation_matrix_arrays(direction)).dot(array(
              [np.random.normal(0, jitter), np.random.normal(0, jitter), 0]))
    self._ray = Ray(direction, initial_position)
    self._history = [self._ray]
    self.elements = []

  def _next_hit(self):
    hits = ((item, item.next_hit(self._ray)) for item in self.elements)
    try:
      return sorted(
          [(hit[0], hit[1], item) for item, hit in hits if hit is not None])[0]
    except IndexError:
      return None

  def propagate(self):
    current_region = Air()
    next_region = None
    while True:
      try:
        tti, bound, element = self._next_hit()
      except TypeError:
        # That's all, folks!
        return
      self._ray = self._ray.propagate(tti)
      self._history.append((tti, self._ray))
      if self._ray.direction.dot(bound.normal(self._ray.position)) > 0:
        next_element = Air()
      else:
        next_region = element
      self._ray = bound.propagate(
          self._ray, current_region.index, next_region.index)
      self._history.append((tti, self._ray))
      current_region = next_region


