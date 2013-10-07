"""
A module for tracking and parameterizing a beam segment in 3D space.

"""

import itop.math.optics as optics
from numpy import array, dot
from numpy.linalg import lstsq, norm
from itop.math import Vector
import copy

class Beam(object):
  """For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.

  The class stores a number of beam-center position samples. Samples must be
  in the same coordinate system, typically that of the tracker used to take
  the samples. Samples may be of any vector-like 3D iterable,
  including the itop.Vector class.

  When 2 or more samples are added to the beam via the add_sample() method,
  the direction of the beam and it's intercept with the z=0 plane are
  computed and stored.

  The fitted direction is always in the positive z direction.

  """
  def __init__(self, z_direction=1):
    """Constructor for beam. Takes an optional z_direction={+1, -1} to
    indicate in which direction the beam is propagating as the fitted
    direction is always in the +z direction.

    """
    self.direction = None
    self.intercept = None
    self.samples = []
    self.power = None
    self._samples_in_fit = 0
    self.distortions = None
    self.z_direction = z_direction

  def add_sample(self, sample):
    """Adds a beam-centroid position sample to the beam samples and updates
    the data.

    Samples may be any 3D vector-like container in the form [x, y, z] where
    the coordinates specify the beam centroid position in a consistant
    coordinate system. Ideally, the samples would be in the form of
    itop.Vector(centroid, error) objects.

    """
    self.samples.append(Vector(sample))
    self.update()

  def __repr__(self):
    return "beam({})".format(self.first_sample().array().tolist())

  def translate(self, displacement):
    """Returns the beam translated by the given displacement vector.

    """
    output = copy.deepcopy(self)
    output.samples = [i + displacement for i in self.samples]
    output.update(force=True)
    return output

  def transform(self, matrix):
    """Returns the beam beam transformed by the given transformation matrix.

    """
    output = copy.deepcopy(self)
    samples = [Vector(dot(matrix, i)) for i in self.samples]
    output.samples = samples
    output.update(force=True)
    return output

  def last_sample(self):
    """Returns the last collected sample."""
    try:
      return self.samples[-1]
    except IndexError:
      return None

  def first_sample(self):
    """Returns the first collected sample."""
    try:
      return self.samples[0]
    except IndexError:
      return None

  def _increasing_z_indexes(self):
    """Returns a list sample indices sorted by increasing z-coordinate."""
    return array([i[2].value for i in self.samples]).argsort().tolist()

  def max_z_sample(self):
    """Returns the sample with the highest z-coordinate."""
    return self.samples[self._increasing_z_indexes()[-1]]

  def min_z_sample(self):
    """Returns the sample with the smallest z-coordinate."""
    return self.samples[self._increasing_z_indexes()[0]]

  def upstream_sample(self):
    """Returns the most upstream sample."""
    return self.samples[
        self._increasing_z_indexes()[(self.z_direction - 1) / 2]]

  def downstream_sample(self):
    """Returns the most downstream sample."""
    return self.samples[
        self._increasing_z_indexes()[(1 + self.z_direction) / -2]]

  def position(self, ordinate, dimension=2):
    """Returns the position in space at the given ordinate in the given
    dimension. Returns None if the position cannot be computed.

    The default dimension is the z axis, i.e. position(z) = (x, y, z)

    """
    if self.direction is not None:
      if dimension == 2:
        return Vector(
            (self.direction/self.direction[dimension]) *
            ordinate + self.intercept)
      else:
        fit = self.fit(dimension)
        length_through_tracker = norm(fit[0][0])
        normal = fit[0][0] / length_through_tracker
        angular_resolution = (
            [fit[1] / length_through_tracker] if fit[1].size > 0 else 0.002)
        return Vector(normal, angular_resolution)
    else:
      return None

  def azimuth(self):
    """Return the azimuthal angle of the beam."""
    return self.direction.project([0, 2]).normalize()[0]

  def update(self, force=False):
    """Updates the beam trajectory information given the samples taken."""
    if ((len(self.samples) > 1) and (force or
        (len(self.samples) != self._samples_in_fit))):
      fit = self.fit()
      length_through_tracker = norm(fit[0][0])
      normal = fit[0][0] / length_through_tracker
      angular_resolution = (
          [fit[1] / length_through_tracker] if fit[1].size > 0 else 0.002)
      self.direction = Vector(normal, angular_resolution)
      self.intercept = fit[0][1]
      self._samples_in_fit = len(self.samples)

  def fit(self, dimension=2):
    """Fits the sampled trajectory to a line using a least-squares
    regression. Returns an array in the form:

      [fit_parameters, residuals, ordinates_rank, ordinates_singular_values]

    The fit parameters of of the form [m, r0] for the vector equation
    r = m*x + r0 where x is the scalar coordinate of the given dimension.

    If no trajectory data is available, None is returned.

    """
    if len(self.samples) > 1:
      try:
        ordinates = [[i[dimension].value, 1.0] for i in self.samples]
        data = [i.array() for i in self.samples]
        return lstsq(ordinates, data)
      except AttributeError:
        ordinates = [[i[dimension], 1.0] for i in self.samples]
        data = [i for i in self.samples]
        return lstsq(ordinates, data)
    else:
      return None

  def refract(self, normal, index_0, index_1):
    """Return the beam direction refracted through a boundary with given
    normal vector and indexes of refraction."""
    return optics.refract(self.direction, normal, index_0, index_1)
