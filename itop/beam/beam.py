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
    self.intercepts = [None, None]
    self.intercept_uncertainties = [None, None]
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
        position = (self.intercepts[0] + fraction * self.slope).tolist()
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
    centroids = []
    if self.camera.read()['power'] >= self.power_level:
      for i in range(samples):
        readout = self.camera.read()
        centroids.append([readout['centroid_x'], readout['centroid_y']])
      centroid = np.mean(zip(*centroids), 1) / 1000.0
      error = np.std(zip(*centroids), 1) / 1000.0
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
    jitter = self.jitter(1)
    if jitter is None:
      return None
    # Do quick sampling to get centroid close to center.
    while abs(jitter[0][0]) > 0.5:
      stage_position = self.controller.groupPosition(self.group_id)
      if len(stage_position) == 2: # If no y-axis stage,
        stage_position.insert(1, 0.0) # Add a y offset
      centroid = list(jitter[0]) + [0]
      position = np.array(stage_position) + np.array(centroid)
      self.controller.groupMoveLine(
          self.group_id, position[0::2], wait=True)
      jitter = self.jitter(1)
    while True:
      jitter = self.jitter(10)
      stage_position = self.controller.groupPosition(self.group_id)
      if len(stage_position) == 2: # If no y-axis stage,
        stage_position.insert(1, 0.0) # Add a y offset
      centroid = list(jitter[0]) + [0]
      position = np.array(stage_position) + np.array(centroid)
      if abs(jitter[0][0]) > abs(jitter[1][0]):
        self.controller.groupMoveLine(
            self.group_id, position[0::2], wait=True)
      else:
        # TODO - Get full stage uncertainties.
        return [position.tolist(), list(jitter[1]) + [0.0005]]

  def findBeam(self, z_position, starting_x_point=-125.0, reverse=False):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(
        self.group_id, [starting_x_point, z_position], wait=True)
    group_configuration = self.controller.groupConfiguration(self.group_id)
    self.controller.groupDelete(self.group_id)

    # Scan for beam crossing.
    scan_axis = self.controller.axes[group_configuration['axes'][0] - 1]
    scan_axis.velocity(10)
    scan_axis.position(125)
    beam_positions = []
    while scan_axis.isMoving():
      if (self.camera.read()['power'] > self.power_level):
        beam_positions.append([scan_axis.position(), z_position])
      elif beam_positions:
        scan_axis.stop(wait=True)
    self.controller.groupCreate(**group_configuration)
    self.controller.pauseForGroup(self.group_id)
    time.sleep(0.125)
    if not beam_positions:
      print "Beam not seen!"
      return None
    # Go back to the beam.
    beam_x = np.mean(beam_positions, 0)[0]
    self.controller.groupMoveLine(self.group_id, [beam_x, z_position], wait=True)
    self.controller.groupVelocity(self.group_id, 10)
    while True:
      centered = self.centerBeam()
      if centered is None:
        self.controller.groupMoveLine(
            self.group_id, [beam_x - 3.0, z_position], wait=True)
      elif centered:
        return centered

  def findTrajectory(self, starting_x_point=-125):
    """
    Find trajectory of single beam.
    """
    # Find the beam at most upstream position.
    self.intercepts[0] = None
    intercept = self.findBeam(-125, starting_x_point=starting_x_point)
    if intercept is not None:
      self.intercepts[0] = np.array(intercept[0])
    else:
      for i in range(3):
        intercept = self.findBeam(-125)
        if intercept is not None:
          self.intercepts[0] = np.array(intercept[0])
          break
      else:
        print "Cannot find beam. Check beam power and camera height."
        return None
    self.intercept_uncertainties[0] = intercept[1]

    # Calculate rough trajectory of the beam.
    step = self.intercepts[0] + np.array([0, 0, 30])
    self.controller.groupMoveLine(self.group_id, step[0::2], wait=True)
    while True:
      centered = self.centerBeam()
      if centered is None:
        step = step + np.array([0, 0, -10])
        self.controller.groupMoveLine(self.group_id, step[0::2], wait=True)
      elif centered:
        break
    upstream_sample = self.position()
    downstream_sample = [self.intercepts[0][0] + (
        ((self.upper_limit_z - self.intercepts[0][2]) /
        (upstream_sample[2] - self.intercepts[0][2])) *
        (upstream_sample[0] - self.intercepts[0][0])),
        self.intercepts[0][1],
        self.upper_limit_z]
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(
        self.group_id, downstream_sample[0::2], wait=True)
    self.controller.groupVelocity(self.group_id, 10)

    # Refine trajectory of the beam.
    while True:
      centered = self.centerBeam()
      if centered is None:
        # TODO - Cross this bridge when we get there.
        pass
      elif centered:
        self.intercepts[1] = np.array(centered[0])
        self.intercept_uncertainties[1] = centered[1]
        self.slope = self.intercepts[1] - self.intercepts[0]
        return self.slope

  def slopeUncertainty(self):
    """
    Returns a list of the four 1 sigma alternate trajectories based on the
    upstream and downstream intercept uncertainties.
    """
    if not self.intercept_uncertainties:
      return None
    signs = [[ 1,  1, -1, -1],
             [ 1, -1, -1,  1],
             [-1, -1,  1,  1],
             [-1,  1,  1, -1]]
    alternates = []
    for i in range(4):
      r0 = np.array(self.intercepts[0])
      rf = np.array(self.intercepts[1])
      r0[0] += signs[i][0] * self.intercept_uncertainties[0][0]
      r0[1] += signs[i][1] * self.intercept_uncertainties[0][1]
      rf[0] += signs[i][2] * self.intercept_uncertainties[1][0]
      rf[1] += signs[i][3] * self.intercept_uncertainties[1][1]
      alternates.append((itlin.normalize(rf - r0)).tolist())
    return alternates

  def moveOnBeam(self, fraction):
      """
      Moves the stage group along the beam trajectory to the given fraction
      of available group path.
      """
      self.controller.groupMoveLine(self.group_id,
          (self.position(fraction))[0::2])

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
