import numpy as np
import itop.math as im
import time
import math

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
    self.slope = None
    self.slope3D = None # (x,y,z)

  def position(self, fraction=None):
    """
    Returns the 3D position of the beam at the current 2D stage position.

    If given an optional argument 'fraction', returns the coordinates of the
    beam at the given fraction of the available stage group trajectory.
    """
    position = None
    if fraction is not None:
      try:
        position = (self.r_initial + fraction * self.slope3D).tolist()
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

  def centerBeam(self):
    """
    Centers the camera on the beam if beam is visible.
    """
    if self.camera.read()['power'] >= self.power_level:
      jitter = [self.camera.read()['centroid_x'] for i in range(10)]
      beam_x = np.mean(jitter)
      # TODO - Change addition or subtraction based on camera direction.
      self.controller.groupMoveLine(self.group_id, (
          np.array(self.controller.groupPosition(self.group_id)) +
          np.array([beam_x / 1000.0, 0])), wait=True)
      return True
    else:
      print "ERROR: Insufficient exposed power."
      return False

  def findBeam(self, position):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, [-125, position], wait=True)
    time.sleep(0.125)
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
    self.controller.groupMoveLine(self.group_id, downstream_sample[0::2], wait=True)
    self.controller.groupVelocity(self.group_id, 10)

    # Refine trajectory of the beam.
    self.centerBeam()
    self.r_final = np.array(self.position())
    self.slope3D = self.r_final - self.r_initial
    self.slope = self.slope3D[0::2]
    return self.slope3D

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
    d = sign * im.linalg.normalize(self.slope3D)
    # The alpha angle is signed and related to the xz-projection.
    alpha = np.arcsin(im.linalg.normalize(d[0::2])[0])
    # The polar angle about y doesn't change with rotations about y, thus:
    beta = -np.arcsin(d[1])
    gamma = 0.0;
    return alpha, beta, gamma
