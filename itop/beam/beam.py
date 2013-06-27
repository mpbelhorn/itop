"""
A module for tracking and parameterizing a beam segment in 3D space.
"""

import numpy as np
import itop.math.linalg as itlin
import time
from collections import namedtuple

TrajectoryData = namedtuple('TrajectoryData',
    ['slope',
     'upstream_point',
     'upstream_error',
     'downstream_point',
     'downstream_error'])

class Beam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """

  def __init__(self, tracker):
    """
    Initialize a constraint on the movement of a LBP on a stage group to travel
    along a beam.

    """
    self.tracker = tracker
    self.slope = None
    self.upstream_point = None
    self.upstream_error = None
    self.downstream_point = None
    self.downstream_error = None

  def position(self, fraction):
    """
    Returns the position in space at a given fraction along path length.
    """
    if self.slope is not None:
      return (np.array(self.upstream_point) +
          fraction * np.array(self.slope)).tolist()
    else:
      return None

  def trajectory(self):
    """
    Returns the beam trajectory data as a named tuple with the same names as
    in the class instance.
    """
    data = [self.slope,
            self.upstream_point,
            self.upstream_error,
            self.downstream_point,
            self.downstream_error]
    if None not in data:
        return TrajectoryData(*data)
    else:
      # Must be default values. Why save them anyways?
      return TrajectoryData(*[None] * 5)

  def load(self, data):
    """
    Restores the trajectory data from a list or tuple of the type returned by
    trajectory().
    """
    if None in data:
      self.slope = None
      self.upstream_point = None
      self.upstream_error = None
      self.downstream_point = None
      self.downstream_error = None
    else:
      self.slope = np.array(data[0])
      self.upstream_point = np.array(data[1])
      self.upstream_error = np.array(data[2])
      self.downstream_point = np.array(data[3])
      self.downstream_error = np.array(data[4])

  def jitter(self, number_of_samples=5):
    """
    Returns the average position in mm of the beam centroid and its
    standard error in the profiler frame after a number of samples if
    the beam is visible, otherwise None is returned.
    """
    first_centroid = self.tracker.centroid()
    if first_centroid is None:
      return None
    else:
      centroids = []
      centroids.append(first_centroid)
      for i in range(number_of_samples - 1):
        centroids.append(self.tracker.centroid())
      centroid = np.mean(zip(*centroids), 1)
      error = np.std(zip(*centroids), 1)
      output = [centroid, error]
      return output

  def centerBeam(self):
    """
    Centers the camera on the beam if beam is visible. If the beam is already
    centered, the function returns the position of the beam in the relative
    to the stage group home + uncertainty. If the beam is visible and not
    centered, the camera is moved to the center. If the beam is not visible,
    None is returned.
    """
    centroid = self.tracker.centroid()
    if centroid is None:
      return None
    # Do quick sampling to get centroid close to center.
    while map(abs, self.tracker.centroid()) > [0.5, 0.5, 0.5]:
      stage_position = self.tracker.stagePosition()
      centroid = self.tracker.centroid()
      position = np.array(stage_position) + np.array(centroid)
      self.tracker.stagePosition(position, wait=True)
    # TODO - Replace following while loop with a finite number of iterations?
    while True:
      jitter = self.jitter()
      stage_position = self.tracker.stagePosition()
      centroid = jitter[0]
      position = np.array(stage_position) + np.array(centroid)
      if any(map(abs, jitter[0]) > jitter[1]):
        self.tracker.stagePosition(position, wait=True)
      else:
        # TODO - Get full stage uncertainties.
        return [position.tolist(), jitter[1] + [0.0005]]

  def findBeam(self, x0=-125, y0=12.5, z0=-125, scan_direction_x=1):
    """
    Scans in X for single beam and centers camera on beam.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.
    """
    x_axis = self.tracker.driver.axes[self.tracker.axes[0] - 1]
    beam_positions = []
    overshoot = scan_direction_x * 2.0
    if not self.tracker.beamVisible():
      # Move camera into starting point.
      self.tracker.groupState(1, fast=True)
      self.tracker.stagePosition([x0, y0, z0], wait=True)

      # Scan for beam crossing.
      self.tracker.groupState(1, fast=False)
      x_axis.position(scan_direction_x * 125)
      while x_axis.isMoving():
        current_position = self.tracker.beamPosition()
        if current_position is not None:
          x_axis.stop(wait=True)
          self.tracker.stagePosition(current_position, wait=True)
          break
      else:
        print "Beam not seen!"
        return None

    # The beam should now be in view.
    self.tracker.groupState(3, fast=True)
    while True:
      centered = self.centerBeam()
      if centered is None:
        self.tracker.stagePosition([beam_x - overshoot, y0, z0], wait=True)
      else:
        return centered

  def distortion(self):
    """
    Returns a list of the ratios r(%) = h(%)/w(%) where h(%) and w(%) are the width
    and height of the beam's best fit gaussian profile at the given percentage of
    the maximum profile power.

    The default percentages are 13.5%, 50.0% and 80.0%
    """
    profile = self.tracker.profiler.read()
    return (profile['height_1']/profile['width_1'],
            profile['height_2']/profile['width_2'],
            profile['height_3']/profile['width_3'])

  def findTrajectory(self, x0=-125, y0=12.5, z0=-125,
      scan_direction_x=1, scan_direction_z=1):
    """
    Find trajectory of single beam.

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

    # Find the beam at the first z extreme.
    for i in [0, 1, 2, -1, -2]:
      first_intercept = self.findBeam(x0, y0 + i * 4.0, z0, scan_direction_x)
      if first_intercept is not None:
        break
    else:
      print "Cannot find beam. Check beam power and camera height."
      return None

    # Calculate rough trajectory of the beam.
    step = first_intercept[0] + scan_direction_z * np.array([0, 0, 30])
    self.tracker.groupState(2, fast=True)
    self.tracker.stagePosition(step, wait=True)
    while True:
      centered = self.centerBeam()
      if centered is None:
        step = step - scan_direction_z * np.array([0, 0, 10])
        self.tracker.stagePosition(step, wait=True)
      elif centered:
        break
    first_sample = np.array(self.tracker.stagePosition())
    direction = itlin.normalize(
        first_sample - first_intercept[0])
    # TODO - Replace following ILS250 assumption with something more robust.
    scale = scan_direction_z * 2 * abs(first_intercept[0][2]) / direction[2]
    second_sample = first_intercept[0] + scale * direction
    self.tracker.groupState(1, fast=True)
    self.tracker.stagePosition(
        second_sample.tolist()[:2] + [scan_direction_z * 125], wait=True)
    self.tracker.groupState(3, fast=True)

    # Refine trajectory of the beam.
    while True:
      second_intercept = self.centerBeam()
      if second_intercept is None:
        # TODO - Cross this bridge when we get there.
        pass
      elif second_intercept:
        if scan_direction_z == 1:
          self.upstream_point = np.array(first_intercept[0])
          self.upstream_error = np.array(first_intercept[1])
          self.downstream_point = np.array(second_intercept[0])
          self.downstream_error = np.array(second_intercept[1])
        else:
          self.downstream_point = np.array(first_intercept[0])
          self.downstream_error = np.array(first_intercept[1])
          self.upstream_point = np.array(second_intercept[0])
          self.upstream_error = np.array(second_intercept[1])
        self.slope = (self.downstream_point - self.upstream_point)
        return self.slope

  def slopeUncertainty(self):
    """
    Returns a list of the four 1 sigma alternate trajectories based on the
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
      r0 = self.upstream_point
      rf = self.downstream_point
      r0[0] += signs[i][0] * self.upstream_error[0]
      r0[1] += signs[i][1] * self.upstream_error[1]
      rf[0] += signs[i][2] * self.downstream_error[0]
      rf[1] += signs[i][3] * self.downstream_error[1]
      alternates.append((itlin.normalize(rf - r0)).tolist())
    return alternates

  def moveOnBeam(self, fraction, fast=False):
      """
      Moves the stage group along the beam trajectory to the given fraction
      of available group path.
      """
      self.tracker.groupState(3, fast)
      self.tracker.stagePosition(self.position(fraction))

  def angles(self, reverse=False):
    """
    Returns the yxz-convention (phi, theta, psi) Tait-Bryan angles needed
    to rotate the stage coordinate system into the beam coordinate system.

    Conventionally, the incoming beam x-axis is always taken to be in the
    table/stage xz-plane, and thus psi == 0. This convention is chosen
    to correspond to the beam polarization.
    """
    sign = -1 if reverse else 1
    d = sign * itlin.normalize(self.slope)
    # The alpha angle is signed and related to the xz-projection.
    phi = np.arcsin(itlin.normalize(d[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    theta = -np.arcsin(d[1])
    psi = 0.0
    return phi, theta, psi
