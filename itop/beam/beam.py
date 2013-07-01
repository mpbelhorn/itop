"""
A module for tracking and parameterizing a beam segment in 3D space.

"""

import numpy as np
import itop.math.linalg as itlin
from collections import namedtuple

TrajectoryData = namedtuple('TrajectoryData',
    ['slope',
     'upstream_point',
     'upstream_error',
     'downstream_point',
     'downstream_error',
     'distortions'])


class Beam(object):
  """For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.

  """

  def __init__(self, tracker):
    """Initialize a constraint on the movement of a LBP on a stage group to
    travel along a beam.

    """
    self.tracker = tracker
    self.slope = None
    self.upstream_point = None
    self.upstream_error = None
    self.downstream_point = None
    self.downstream_error = None
    self.distortions = None

  def position(self, fraction):
    """Returns the position in space at a given fraction along path length.

    """
    if self.slope is not None:
      return (np.array(self.upstream_point) +
          fraction * np.array(self.slope)).tolist()
    else:
      return None

  def trajectory(self):
    """Returns the beam trajectory data as a named tuple with the same names as
    in the class instance.

    """
    data = [self.slope,
            self.upstream_point,
            self.upstream_error,
            self.downstream_point,
            self.downstream_error,
            self.distortions]
    if None not in data:
      return TrajectoryData(*data)
    else:
      # Must be default values. Why save them anyways?
      return TrajectoryData(*[None] * 6)

  def load(self, data):
    """Restores the trajectory data from a list or tuple of the type returned by
    trajectory().

    """
    if None in data:
      self.slope = None
      self.upstream_point = None
      self.upstream_error = None
      self.downstream_point = None
      self.downstream_error = None
      self.distortions = None
    else:
      self.slope = np.array(data[0])
      self.upstream_point = np.array(data[1])
      self.upstream_error = np.array(data[2])
      self.downstream_point = np.array(data[3])
      self.downstream_error = np.array(data[4])
      self.distortions = data[5]

  def jitter(self, number_of_samples=5):
    """Returns the average position in mm of the beam centroid and its
    standard error in the profiler frame after a number of samples if
    the beam is visible, otherwise None is returned.

    """
    first_centroid = self.tracker.centroid()
    if first_centroid is None:
      return None
    else:
      centroids = []
      centroids.append(first_centroid)
      for _ in range(number_of_samples - 1):
        centroids.append(self.tracker.centroid())
      centroid = np.mean(zip(*centroids), 1)
      error = np.std(zip(*centroids), 1)
      output = [centroid, error]
      return output

  def center_beam(self):
    """Centers the camera on the beam if beam is visible. If the beam is already
    centered, the function returns the position of the beam in the relative
    to the stage group home + uncertainty. If the beam is visible and not
    centered, the camera is moved to the center. If the beam is not visible,
    None is returned.

    """
    centroid = self.tracker.centroid()
    if centroid is None:
      return None
    # Do quick sampling to get centroid close to center.
    while any((abs(i) > 0.5 for i in self.tracker.centroid())):
      stage_position = self.tracker.stage_position()
      centroid = self.tracker.centroid()
      position = np.array(stage_position) + np.array(centroid)
      self.tracker.stage_position(position, wait=True)
    # TODO - Replace following while loop with a finite number of iterations?
    while True:
      jitter = self.jitter()
      stage_position = self.tracker.stage_position()
      centroid = jitter[0]
      position = np.array(stage_position) + np.array(centroid)
      if any((abs(value) > error for value, error in zip(*jitter))):
        self.tracker.stage_position(position, wait=True)
      else:
        # TODO - Get full stage uncertainties.
        return [position.tolist(), jitter[1] + [0.0005]]

  def find_beam(
      self, start_point=None, scan_direction_x=1):
    """Scans in X for single beam and centers camera on beam.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.

    """
    start_point = start_point or [-125.0, 12.5, -125.0]
    x_axis = self.tracker.driver.axes[self.tracker.axes[0] - 1]
    current_position = None
    overshoot = scan_direction_x * 2.0
    if not self.tracker.beam_visible():
      # Move camera into starting point.
      self.tracker.change_grouping(1, fast=True)
      self.tracker.stage_position(start_point, wait=True)

      # Scan for beam crossing.
      self.tracker.change_grouping(1, fast=False)
      x_axis.position(scan_direction_x * 125)
      while x_axis.is_moving():
        current_position = self.tracker.beam_position()
        if current_position is not None:
          x_axis.stop(wait=True)
          self.tracker.stage_position(current_position, wait=True)
          break
      else:
        print "Beam not seen!"
        return None

    # The beam should now be in view.
    self.tracker.change_grouping(3, fast=True)
    while True:
      centered = self.center_beam()
      if centered is None:
        self.tracker.stage_position(
            np.array(current_position) - np.array([overshoot, 0, 0]),
            wait=True)
      else:
        return centered

  def distortion(self):
    """Returns a list of the ratios r(%) = h(%)/w(%) where h(%) and w(%) are the
    width and height of the beam's best fit gaussian profile at the given
    percentage of the maximum profile power.

    The default percentages are 13.5%, 50.0% and 80.0%

    """
    profile = self.tracker.profiler.read()
    return (profile['height_1']/profile['width_1'],
            profile['height_2']/profile['width_2'],
            profile['height_3']/profile['width_3'])

  def find_trajectory(self, start_point=None,
      scan_direction_x=1, scan_direction_z=1):
    """Find trajectory of single beam.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.
      scan_direction_z (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) z direction.

    """
    # Clear current beam data.
    self.slope = None
    self.upstream_point = None
    self.upstream_error = None
    self.downstream_point = None
    self.downstream_error = None
    self.distortions = None

    # Find the beam at the first z extreme.
    start_point = start_point or [-125.0, 12.5, -125.0]
    for i in [0, 1, 2, -1, -2]:
      first_intercept = self.find_beam(
          np.array(start_point) + np.array([0, i * 4.0, 0]), scan_direction_x)
      first_distortion = self.distortion()
      if first_intercept is not None:
        break
    else:
      print "Cannot find beam. Check beam power and camera height."
      return None

    # Calculate rough trajectory of the beam.
    step = first_intercept[0] + scan_direction_z * np.array([0, 0, 30])
    self.tracker.change_grouping(2, fast=True)
    self.tracker.stage_position(step, wait=True)
    while True:
      centered = self.center_beam()
      if centered is None:
        step = step - scan_direction_z * np.array([0, 0, 10])
        self.tracker.stage_position(step, wait=True)
      elif centered:
        break
    first_sample = np.array(self.tracker.stage_position())
    direction = itlin.normalize(
        first_sample - first_intercept[0])
    # TODO - Replace following ILS250 assumption with something more robust.
    scale = scan_direction_z * 2 * abs(first_intercept[0][2]) / direction[2]
    second_sample = first_intercept[0] + scale * direction
    self.tracker.change_grouping(1, fast=True)
    self.tracker.stage_position(
        second_sample.tolist()[:2] + [scan_direction_z * 125], wait=True)
    self.tracker.change_grouping(3, fast=True)

    # Refine trajectory of the beam.
    while True:
      second_intercept = self.center_beam()
      second_distortion = self.distortion()
      if second_intercept is None:
        # TODO - Cross this bridge when we get there.
        pass
      elif second_intercept:
        if scan_direction_z == 1:
          self.upstream_point = np.array(first_intercept[0])
          self.upstream_error = np.array(first_intercept[1])
          self.downstream_point = np.array(second_intercept[0])
          self.downstream_error = np.array(second_intercept[1])
          self.distortions = (first_distortion, second_distortion)
        else:
          self.downstream_point = np.array(first_intercept[0])
          self.downstream_error = np.array(first_intercept[1])
          self.upstream_point = np.array(second_intercept[0])
          self.upstream_error = np.array(second_intercept[1])
          self.distortions = (second_distortion, first_distortion)
        self.slope = (self.downstream_point - self.upstream_point)
        return self.slope

  def slope_uncertainty(self):
    """Returns a list of the four 1 sigma alternate trajectories based on the
    upstream and downstream intercept uncertainties.

    """
    if None in (self.upstream_error, self.downstream_error):
      return None
    signs = [[ 1,  1, -1, -1],
             [ 1, -1, -1,  1],
             [-1, -1,  1,  1],
             [-1,  1,  1, -1]]
    alternates = []
    for i in range(4):
      new_upstream = self.upstream_point
      new_downstream = self.downstream_point
      new_upstream[0] += signs[i][0] * self.upstream_error[0]
      new_upstream[1] += signs[i][1] * self.upstream_error[1]
      new_downstream[0] += signs[i][2] * self.downstream_error[0]
      new_downstream[1] += signs[i][3] * self.downstream_error[1]
      alternates.append(
          (itlin.normalize(new_downstream - new_upstream)).tolist())
    return alternates

  def move_on_beam(self, fraction, fast=False):
    """Moves the stage group along the beam trajectory to the given fraction
    of available group path.

    """
    self.tracker.change_grouping(3, fast)
    self.tracker.stage_position(self.position(fraction))

  def angles(self, reverse=False):
    """Returns the yxz-convention (phi, theta, psi) Tait-Bryan angles needed
    to rotate the stage coordinate system into the beam coordinate system.

    Conventionally, the incoming beam x-axis is always taken to be in the
    table/stage xz-plane, and thus psi == 0. This convention is chosen
    to correspond to the beam polarization.

    """
    sign = -1 if reverse else 1
    direction = sign * itlin.normalize(self.slope)
    # The alpha angle is signed and related to the xz-projection.
    phi = np.arcsin(itlin.normalize(direction[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    theta = -np.arcsin(direction[1])
    psi = 0.0
    return phi, theta, psi
