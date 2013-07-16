"""
A module for tracking and parameterizing a beam segment in 3D space.

"""

from numpy import arcsin, array
from numpy.linalg import lstsq, norm
from itop.math.linalg import normalize
from itop.math import Vector

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
    self._samples = []
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
    self._samples.append(sample)
    self._update()

  def __repr__(self):
    return repr(self.direction)

  def translate(self, displacement):
    """Returns the beam translated by the given displacement vector."""
    output = Beam(self.z_direction)
    output.distortions = self.distortions
    displacement = Vector(displacement)
    output._samples = [i + displacement for i in self._samples]
    output._update()
    return output

  def align(self, alignment):
    """Returns the beam rotated into the coordinate system described by the
    given alignment.

    """
    output = Beam(self.z_direction)
    output.distortions = self.distortions
    rotation_axes = [[0, 1, 0],
                     [1, 0, 0],
                     [0, 0, 1]]
    yxz_tait_bryan_parameters = zip(rotation_axes, alignment.angles)
    for i in self._samples:
      rotated_sample = Vector(i)
      for axis, angle in yxz_tait_bryan_parameters:
        rotated_sample = rotated_sample.rotate(axis, angle)
      output._samples.append(rotated_sample)
    output._update()
    return output

  def last_sample(self):
    """Returns the last collected sample."""
    return self._samples[-1]

  def first_sample(self):
    """Returns the first collected sample."""
    return self._samples[0]

  def _increasing_z_indexes(self):
    """Returns a list sample indices sorted by increasing z-coordinate."""
    return array([i[2].value for i in self._samples]).argsort().tolist()

  def max_z_sample(self):
    """Returns the sample with the highest z-coordinate."""
    return self._samples[self._increasing_z_indexes()[-1]]

  def min_z_sample(self):
    """Returns the sample with the smallest z-coordinate."""
    return self._samples[self._increasing_z_indexes()[0]]

  def upstream_sample(self):
    """Returns the most upstream sample."""
    return self._samples[
        self._increasing_z_indexes()[(self.z_direction - 1) / 2]]

  def downstream_sample(self):
    """Returns the most downstream sample."""
    return self._samples[
        self._increasing_z_indexes()[(1 + self.z_direction) / -2]]

  def position(self, ordinate, dimension=2):
    """Returns the position in space at the given ordinate in the given
    dimension. Returns None if the position cannot be computed.

    The default dimension is the z axis, i.e. position(z) = (x, y, z)

    """
    if self.direction is not None:
      if dimension == 2:
        return Vector(self.direction * ordinate + self.intercept)
      else:
        # Calculate the trajectory in this dimension and pass the point on.
        pass
    else:
      return None

  def _update(self):
    """Updates the beam trajectory information given the samples taken."""
    if ((len(self._samples) > 1) and
        (len(self._samples) != self._samples_in_fit)):
      fit = self.fit()
      length_through_tracker = norm(fit[0][0])
      normal = fit[0][0] / length_through_tracker
      angular_resolution = (
          [fit[1] / length_through_tracker] if fit[1].size > 0 else 0.002)
      self.direction = Vector(normal, angular_resolution)
      self.intercept = fit[0][1]
      self._samples_in_fit = len(self._samples)

  def fit(self, dimension=2):
    """Fits the sampled trajectory to a line using a least-squares
    regression. Returns an array in the form:

      [fit_parameters, residuals, ordinates_rank, ordinates_singular_values]

    The fit parameters of of the form [m, r0] for the vector equation
    r = m*x + r0 where x is the scalar coordinate of the given dimension.

    If no trajectory data is available, None is returned.

    """
    if len(self._samples) > 1:
      try:
        ordinates = [[i[dimension].value, 1.0] for i in self._samples]
        data = [i.array() for i in self._samples]
        return lstsq(ordinates, data)
      except AttributeError:
        ordinates = [[i[dimension], 1.0] for i in self._samples]
        data = [i for i in self._samples]
        return lstsq(ordinates, data)
    else:
      return None

  def angles(self, reverse=False):
    """Returns the yxz-convention (phi, theta, psi) Tait-Bryan angles needed
    to rotate the tracker coordinate system into the beam coordinate system.

    Conventionally, the incoming beam x-axis is always taken to be in the
    table/stage xz-plane, and thus psi == 0. This convention is chosen
    to correspond to the beam polarization.

    """
    sign = -1 if reverse else 1
    direction = sign * self.direction.array()
    # The alpha angle is signed and related to the xz-projection.
    phi = arcsin(normalize(direction[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    theta = -arcsin(direction[1])
    psi = 0.0
    return phi, theta, psi

