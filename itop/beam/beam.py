"""
A module for tracking and parameterizing a beam segment in 3D space.
"""

import numpy as np
import itop.math as im
import time

class Beam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """

  def __init__(self, controller, group_id, camera, **kwargs):
    """
    Initialize a constraint on the movement of a LBP on a stage group to travel
    along a beam.

    Keyword arguments accepted to constrain the region of group travel.
    Assume group of ILS250CC stages if no kwargs given. Units are those
    that the stages are currently programmed to.

    Option=default values are as follows:
    lower_limit_x=-125 - Lower travel limit.
    lower_limit_z=-125 - Lower travel limit.
    upper_limit_x=125 - Upper travel limit.
    upper_limit_z=125 - Upper travel limit.
    power=level - Beam-in-view power threshold.
    """
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    self.lower_limit_x = kwargs.pop('lower_limit_x', -125)
    self.upper_limit_x = kwargs.pop('upper_limit_x',  125)
    self.lower_limit_z = kwargs.pop('lower_limit_z', -125)
    self.upper_limit_z = kwargs.pop('upper_limit_z',  125)
    self.power_level = kwargs.pop('power_level', 0.003)
    self.r_initial = None
    self.r_final = None
    self.slope = None # (x,y,z)

  def position(self, fraction=None):
    """
    Returns the 3D position of the beam at the current 2D stage position.

    If given an optional argument 'fraction', returns the coordinates of the
    beam at the given fraction of the available stage group trajectory.
    """
    position = None
    if fraction is not None:
      try:
        position = (self.r_initial + fraction * self.slope).tolist()
      except TypeError:
        print "Trajectory is not initilized"
    else:
      profile = self.camera.read()
      if (profile['power'] >= self.power_level):
        position = self.controller.groupPosition(self.group_id)
        elevation = profile['centroid_y'] / 1000.0
        position.insert(1, elevation)
      else:
        print "Beam not visible"
    return position

  def jitter(self, samples=10):
    """
    Returns the average position in mm of the beam centroid and its
    standard error in the camera frame after a number of samples if
    the beam is visible, otherwise None is returned.
    """
    output = None
    positions = []
    if self.camera.read()['power'] >= self.power_level:
      for i in range(samples):
        readout = self.camera.read()
        positions.append([readout['centroid_x'], readout['centroid_y']])
      position = np.mean(zip(*positions)) / 1000.0
      error = np.std(zip(*positions)) / 1000.0
      output = [position, error]
    return output

  def centerBeam(self):
    """
    Centers the camera on the beam if beam is visible.
    """
    jitter = self.jitter()
    if jitter is not None:
      # The HDLBP software axis orientation must be correctly set for
      # the direction the camera is facing relative to the stage.
      self.controller.groupMoveLine(self.group_id, (
          np.array(self.controller.groupPosition(self.group_id)) +
          np.array(jitter[0][0], 0)), wait=True)
      return True
    else:
      return False

  def findBeam(self, position):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, [-125, position], wait=True)
    group_configuration = self.controller.groupConfiguration(self.group_id)
    self.controller.groupDelete(self.group_id)

    # Scan for beam crossing.
    scan_axis = self.controller.axes[group_configuration['axes'][0] - 1]
    scan_axis.velocity(10)
    scan_axis.position(125)
    beam_positions = []
    while scan_axis.isMoving():
      if (self.camera.read()['power'] > self.power_level):
        beam_positions.append([scan_axis.position(), position])
      elif beam_positions:
        scan_axis.stop(wait=True)
    self.controller.groupCreate(**group_configuration)
    self.controller.pauseForGroup(self.group_id)
    time.sleep(0.125)
    if not beam_positions:
      print "Beam not seen!"
      return None
    # Go back to the beam.
    print beam_positions
    beam_x = np.mean(beam_positions, 0)[0]
    self.controller.groupMoveLine(self.group_id, [beam_x, position], wait=True)
    self.controller.groupVelocity(self.group_id, 10)
    if not self.centerBeam():
      self.controller.groupMoveLine(
          self.group_id, [beam_x - 3.0, position], wait=True)
      if not self.centerBeam():
        self.controller.groupMoveLine(
            self.group_id, [beam_x + 6.0, position], wait=True)
    if self.centerBeam():
      print "Beam found"
    return self.position()

  def findTrajectory(self):
    """
    Find trajectory of single beam.
    """
    # Find the beam at most upstream position.
    self.r_initial = None
    for i in range(3):
      intercept = self.findBeam(-125)
      if intercept is not None:
        self.r_initial = np.array(intercept)
        break
    else:
      print "Cannot find beam. Check beam power and camera height."
      return None

    # Calculate rough trajectory of the beam.
    step = self.r_initial + np.array([0, 0, 30])
    self.controller.groupMoveLine(self.group_id, step[0::2], wait=True)
    while not self.centerBeam():
      step = step - np.array([0, 0, 10])
      self.controller.groupMoveLine(self.group_id, step[0::2], wait=True)
    upstream_sample = self.position()
    downstream_sample = [self.r_initial[0] + (
        ((self.upper_limit_z - self.r_initial[2]) /
        (upstream_sample[2] - self.r_initial[2])) *
        (upstream_sample[0] - self.r_initial[0])),
        self.r_initial[1],
        self.upper_limit_z]
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(
        self.group_id, downstream_sample[0::2], wait=True)
    self.controller.groupVelocity(self.group_id, 10)

    # Refine trajectory of the beam.
    self.centerBeam()
    self.r_final = np.array(self.position())
    self.slope = self.r_final - self.r_initial
    return self.slope

  def moveOnBeam(self, fraction):
      """
      Moves the stage group along the beam trajectory to the given fraction
      of available group path.
      """
      self.controller.groupMoveLine(self.group_id,
          (self.position(fraction))[0::2])

  def angles(self, reverse=False):
    """
    Returns the yxz-convention (alpha, beta, gamma) Euler angles needed to
    rotate the stage coordinate system into the beam coordinate system.

    Conventionally, the incoming beam x-axis is always taken to be in the
    table/stage xz-plane, and thus gamma == 0. This convention is chosen
    to correspond to the beam polarization.
    """
    sign = -1 if reverse else 1
    d = sign * im.linalg.normalize(self.slope)
    # The alpha angle is signed and related to the xz-projection.
    alpha = np.arcsin(im.linalg.normalize(d[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    beta = -np.arcsin(d[1])
    gamma = 0.0
    return alpha, beta, gamma
