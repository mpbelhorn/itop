"""
A module for tracking and parameterizing a beam segment in 3D space.
"""

import numpy as np
import itop.math.linalg as itlin
import time

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
    self.trajectory = {
        'slope': None,
        'upstream point': None,
        'upstream error': None,
        'downstream point': None,
        'downstream error': None,
        }

  def position(self, fraction=None):
    """
    Returns the 3D position of the beam at the current 2D stage position.

    If given an optional argument 'fraction', returns the coordinates of the
    beam at the given fraction of the available stage group trajectory.
    """
    position = None
    if fraction is not None:
      try:
        position = (self.trajectory['upstream point'] +
            fraction * self.trajectory['slope']).tolist()
      except TypeError:
        print "Trajectory is not initilized"
    else:
      position = self.tracker.beamPosition()
    return position

  def dump(self):
    """
    Returns the beam trajectory data as a serializable dictionary
    of lists instead of numpy array-types.

    The dictionary keys are the same name as the instance variables
    (intercepts, intercept_uncertainties, slope)
    """
    data = []
    try:
      data = [(k, v.tolist()) for k, v in self.trajectory.items()]
    except AttributeError:
      # Must be default values. Why save them anyways?
      data = [(k, None) for k, v in self.trajectory.items()]
    return data

  def load(self, data):
    """
    Restores the trajectory data from a list of the type returned by
    dump().
    """
    try:
      if None in [v for k, v in data]:
        print "Data is incomplete. Values unchanged."
      else:
        self.trajectory = dict([(k, np.array(v)) for k, v in data])
    except TypeError:
      print "Data is invalid. Values unchanged."

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

  def findBeam(self, x0=-125, y0=12.5, z0=-125, reverse=False):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.tracker.groupState(1, fast=True)
    self.tracker.stagePosition([x0, y0, z0], wait=True)

    # Scan for beam crossing.
    self.tracker.groupState(1, fast=False)
    x_axis = self.tracker.driver.axes[self.tracker.axes[0] - 1]
    x_axis.position(125)
    beam_positions = []
    while x_axis.isMoving():
      current_position = self.tracker.beamPosition()
      if current_position is not None:
        beam_positions.append(current_position)
      elif beam_positions:
        x_axis.stop(wait=True)
    if not beam_positions:
      print "Beam not seen!"
      return None
    # Go back to the beam.
    beam_x = np.mean(beam_positions, 0)[0] - 2.0
    self.tracker.stagePosition([beam_x, y0, z0], wait=True)
    self.tracker.groupState(3, fast=True)
    while True:
      centered = self.centerBeam()
      if centered is None:
        self.tracker.stagePosition([beam_x - 3.0, y0, z0], wait=True)
      else:
        return centered

  def findTrajectory(self, x0=-125, y0=12.5, z0=-125, reverse=False):
    """
    Find trajectory of single beam.
    """
    # Find the beam at most upstream position.
    self.trajectory['upstream point'] = None
    self.trajectory['upstream error'] = None
    self.trajectory['downstream point'] = None
    self.trajectory['downstream error'] = None
    first_point = 'upstream point'
    first_error = 'upstream error'
    second_point = 'downstream point'
    second_error = 'downstream error'
    if reverse:
      first_point = 'downstream point'
      first_error = 'downstream error'
      second_point = 'upstream point'
      second_error = 'upstream error'
    for i in [0, 1, 2, -1, -2]:
      first_intercept = self.findBeam(x0, y0 + i * 4.0, z0)
      if first_intercept is not None:
        self.trajectory[first_point] = np.array(first_intercept[0])
        self.trajectory[first_error] = np.array(first_intercept[1])
        break
    else:
      print "Cannot find beam. Check beam power and camera height."
      return None

    # Calculate rough trajectory of the beam.
    sign = 1 if not reverse else -1
    step = self.trajectory[first_point] + sign * np.array([0, 0, 30])
    self.tracker.groupState(2, fast=True)
    self.tracker.stagePosition(step, wait=True)
    while True:
      centered = self.centerBeam()
      if centered is None:
        step = step - sign * np.array([0, 0, 10])
        self.tracker.stagePosition(step, wait=True)
      elif centered:
        break
    first_sample = np.array(self.tracker.stagePosition())
    direction = itlin.normalize(
        first_sample - self.trajectory[first_point])
    # TODO - Replace following ILS250 assumption with something more robust.
    scale = sign * 2 * abs(self.trajectory[first_point][2]) / direction[2]
    second_sample = self.trajectory[first_point] + scale * direction
    self.tracker.groupState(1, fast=True)
    self.tracker.stagePosition(second_sample.tolist()[:2] + [sign * 125], wait=True)
    self.tracker.groupState(3, fast=True)

    # Refine trajectory of the beam.
    while True:
      second_intercept = self.centerBeam()
      if second_intercept is None:
        # TODO - Cross this bridge when we get there.
        pass
      elif second_intercept:
        self.trajectory[second_point] = np.array(second_intercept[0])
        self.trajectory[second_error] = np.array(second_intercept[1])
        self.trajectory['slope'] = (self.trajectory['downstream point'] -
            self.trajectory['upstream point'])
        return self.trajectory['slope']

  def slopeUncertainty(self):
    """
    Returns a list of the four 1 sigma alternate trajectories based on the
    upstream and downstream intercept uncertainties.
    """
    if (self.trajectory['upstream error'] is None
        or self.trajectory['downstream error'] is None):
      return None
    signs = [[ 1,  1, -1, -1],
             [ 1, -1, -1,  1],
             [-1, -1,  1,  1],
             [-1,  1,  1, -1]]
    alternates = []
    for i in range(4):
      r0 = self.trajectory['upstream point']
      rf = self.trajectory['downstream point']
      r0[0] += signs[i][0] * self.trajectory['upstream error'][0]
      r0[1] += signs[i][1] * self.trajectory['upstream error'][1]
      rf[0] += signs[i][2] * self.trajectory['downstream error'][0]
      rf[1] += signs[i][3] * self.trajectory['downstream error'][1]
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
    d = sign * itlin.normalize(self.trajectory['slope'])
    # The alpha angle is signed and related to the xz-projection.
    phi = np.arcsin(itlin.normalize(d[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    theta = -np.arcsin(d[1])
    psi = 0.0
    return phi, theta, psi
