# -*- coding: utf-8 -*-
"""
A module for tracking the position of beam segments in 3D space.

"""
from numpy import array, mean, std, arange
from itop.beam import Beam
from itop.math import Vector
import math as sys_math

class TrackerError(Exception):
  """Error and exception handling for a beam tracker."""
  pass

class Tracker(object):
  """A class to represent an HD-LBP on a set of ESP stages.

  """
  # Static configurations dictionary.
  configurations = {
      'Fast ILS': {'velocity': 40.0, 'acceleration': 50, 'deceleration': 50},
      'Slow ILS': {'velocity': 10.0, 'acceleration': 50, 'deceleration': 50},
      'Fast LTA': {'velocity':  5.0, 'acceleration': 20, 'deceleration': 20},
      'Slow LTA': {'velocity':  2.0, 'acceleration': 20, 'deceleration': 20},
      }

  def __init__(self, driver, rotation_stage, profiler, beam_monitor, **kwargs):
    """Constructor for beam Tracker.

    The tracker needs a reference to an externally defined stage driver and
    beam profiler.

    The stage driver must be able to control the x,y,z position of the beam
    profiler. By default, the driver axes are taken to correspond to:
      axis 1 <-> x-dimension
      axis 2 <-> y-dimension
      axis 3 <-> z-dimension
    This can be overwritten by supplying the optional xyz_axes argument.

    The profiler must be properly configured to output the correct beam power
    for the given filters and base power. The coordinate system must also be
    flipped about the horizontal if the LBP is mounted upside-down.

    In order to calculate beam reflection angles correctly, the tracker axes
    alignment with respect to the beam splitter output must be established.
    One of the following musth happen:
      1.) The path to previously saved and still valid alignment data must
          be supplied at tracker object construction,
      2.) The tracker.load_alignment method must be called manually,
      3.) or the tracker.align() method must be called.

    Any keyword arguments passed that are not handled below are passed to
    the group creation routine.
    Optional keyword arguments:
      'xyz_axes' ([1,2,3]): Sets the axis-dimension map in the order [x,y,z].
      'alignment' (None): Set an external alignment configuration.
      'facing_z_direction' (-1): Direction camera is facing in z. Must be ±1.

    """
    self.devices = {
        'driver':driver,
        'r_stage':rotation_stage,
        'profiler':profiler,
        'monitor':beam_monitor,
        }
    xyz_axes = kwargs.pop('xyz_axes', [1, 2, 3])
    self.axes = (driver.axes[xyz_axes[0] - 1],
                 driver.axes[xyz_axes[1] - 1],
                 driver.axes[xyz_axes[2] - 1])
    self.group_state = 1 # 1=axes independent, 2=xz grouped, 3=xyz grouped
    self.facing_z_direction = kwargs.pop('facing_z_direction', -1)
    self.power_index = None

    # Optional instance variables.
    self.group_id = kwargs.pop('group_id', 1)

    self.change_grouping(1, fast=True)


  def change_grouping(self, state=1, fast=False):
    """Groups the tracker stages into one of 3 modes given by the 'state'
    argument. If no state is given, stages are ungrouped.

    Possible states are:
    1: Ungrouped   - Stages are ungrouped and can be moved independently.
    2: XZ Grouped  - X and Z axes are grouped together, Y is independent.
    3: XYZ Grouped - All stages are grouped together. Group kinematics are
                     limited by the slowest stage in the group.

    This function takes the same keyword arguments as
    'itop.motioncontrol.controller.group_create'.

    """
    ils_configuration = Tracker.configurations[
        'Fast ILS' if fast else 'Slow ILS']
    lta_configuration = Tracker.configurations[
        'Fast LTA' if fast else 'Slow LTA']
    axis_ids = [axis.axis_id[0] for axis in self.axes]
    if state == 3:
      kwargs = lta_configuration
      self.devices['driver'].group_create(axis_ids, **kwargs)
      self.group_state = 3
    elif state == 2:
      kwargs = ils_configuration
      self.devices['driver'].group_create(axis_ids[0::2], **kwargs)
      self.group_state = 2
    else:
      self.devices['driver'].group_delete(self.group_id)
      self.axes[0].velocity(ils_configuration['velocity'])
      self.axes[0].acceleration(ils_configuration['acceleration'])
      self.axes[0].deceleration(ils_configuration['deceleration'])
      self.axes[1].velocity(lta_configuration['velocity'])
      self.axes[1].acceleration(lta_configuration['acceleration'])
      self.axes[1].deceleration(lta_configuration['deceleration'])
      self.axes[2].velocity(ils_configuration['velocity'])
      self.axes[2].acceleration(ils_configuration['acceleration'])
      self.axes[2].deceleration(ils_configuration['deceleration'])
      self.group_state = 1

  def rotate(angle, wait=False):
    """Rotate the profiler to the given angle and set any z-environment
    flags.

    """
    self.devices['r_stage'].position(angle, wait)
    facing_z_direction = sys_math.copysign(
            1.0, sys_math.cos(sys_math.radians(angle)))


  def position(self, xyz_coordinates=None, wait=False):
    """Returns the stage position of the stage. If passed a set of coordinates,
    also moves stage to that position.

    """
    def xyz_grouped(coords):
      """Action to take if all 3 stages are grouped."""
      if coords is None:
        return self.devices['driver'].group_position(self.group_id)
      else:
        self.devices['driver'].group_move_line(self.group_id, coords, wait=wait)

    def xz_grouped(coords):
      """Action to take if only x-z stages are grouped."""
      if coords is None:
        position = self.devices['driver'].group_position(self.group_id)
        position = position.project([0, None, 1])
        position[1] = self.axes[1].position()
        return position
      else:
        self.devices['driver'].group_move_line(self.group_id, coords[0::2])
        self.axes[1].position(coords[1])
        if wait:
          self.devices['driver'].pause_for_stages()

    def ungrouped(coords):
      """Action to take if no stages are grouped."""
      if coords is None:
        return Vector([self.axes[0].position(),
                       self.axes[1].position(),
                       self.axes[2].position()])
      else:
        self.axes[0].position(coords[0])
        self.axes[1].position(coords[1])
        self.axes[2].position(coords[2])
        if wait:
          self.devices['driver'].pause_for_stages()

    cases = {
        1: ungrouped,
        2: xz_grouped,
        3: xyz_grouped,
        }

    return cases[self.group_state](xyz_coordinates)


  def _centroid(self, samples=1):
    """Return the (x, y, z) coordinates of the centroid in the profiler
    profiler coordinate system. The z coordinate is always 0, but included so
    the centroid position can be directly added to stage coordinate positions.

    If the beam is not in view, None is returned.

    """
    profiles = []
    for _ in range(samples):
      profile = self.devices['profiler'].profile(level=self.power_index)
      if profile is None:
        return None
      profiles.append(profile)
    centroids = [(-1 * self.facing_z_direction * i['centroid_x'],
                 i['centroid_y'], 0.0) for i in profiles]
    centroid = mean(zip(*centroids), 1)
    error = [std(zip(*centroids), 1)] if samples > 1 else 0.030
    return Vector(centroid, error)

  def get_beam_position(self, samples=1):
    """If visible, returns the position of the beam centroid in the tracker
    coordinate system. Otherwise returns None

    """
    centroid = self._centroid(samples)
    if centroid is None:
      return None
    else:
      return self.position() + centroid

  def center_beam(self):
    """Centers the camera on the beam if beam is visible. If the beam is
    already centered, the function returns the position of the beam in the
    relative to the stage group home + uncertainty. If the beam is visible
    and not centered, the camera is moved to the center. If the beam is not
    visible, None is returned.

    """
    centroid = self._centroid()
    if centroid is None:
      return None
    # Do quick sampling to get centroid close to center.
    while centroid != Vector([0.0, 0.0, 0.0], 0.050):
      rough_position = self.position() + centroid
      self.position(rough_position, wait=True)
      centroid = self._centroid()
      for dim, coord in enumerate(rough_position):
        if ((coord.value > self.axes[dim].limits.upper + 0.05) or
            (coord.value < self.axes[dim].limits.lower - 0.05)):
          raise TrackerError(
             'Cannot measure beam outside stage limits.')
    # Perhaps replace following while loop with a finite number of iterations?
    while True:
      centroid = self._centroid(8)
      centered_position = centroid + self.position()
      for dim, coord in enumerate(centered_position):
        if ((coord.value > self.axes[dim].limits.upper + 0.01) or
            (coord.value < self.axes[dim].limits.lower - 0.01)):
          raise TrackerError(
             'Cannot measure beam outside stage limits.')
      if centroid != Vector([0.0, 0.0, 0.0], 0.001):
        self.position(centered_position, wait=True)
      else:
        return centered_position

  def _scan_until_beam_visible(self, axis):
    """While the axis is moving, check for beam position. If beam
    becomes visible, stop the axis and return the position of the beam,
    otherwise return None.
    """
    while axis.is_moving():
      beam_position = self.get_beam_position()
      if beam_position is not None:
        axis.stop(wait=True)
        return beam_position

  def find_beam_center(self, start_point=None, scan_direction_x=1):
    """Scans in X for a single beam and centers it on the CCD.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.

    """
    if start_point is None:
      start_point = [
          self.axes[0].limits.lower,
          self.axes[1].limits.lower,
          self.axes[2].limits.lower]
    x_axis = self.axes[0]
    beam_position = self.get_beam_position()
    if beam_position is not None:
      # Beam is already in view.
      if beam_position[2] != start_point[2]:
        # Capture trajectory and move to correct z position.
        beam = Beam()
        beam.add_sample(self.center_beam())
        delta_z_sign = ((start_point[2] - beam_position[2]) /
            abs(start_point[2] - beam_position[2]))
        self.change_grouping(1, fast=True)
        self.position(beam_position +
            delta_z_sign * array([0, 0, 10]), wait=True)
        beam.add_sample(self.center_beam())
        beam_position = self.position(
            beam.position(start_point[2]), wait=True)
    else:
      # Beam not in view. Move camera into full starting point.
      self.change_grouping(1, fast=True)
      self.position(start_point, wait=True)
      self.change_grouping(1, fast=False)
      number_of_scans = 0
      while beam_position is None:
        # Scan for beam crossing.
        x_axis.position(self.axes[0].limits.direction(scan_direction_x))
        beam_position = self._scan_until_beam_visible(x_axis)
        if number_of_scans * 3.0 > self.axes[1].limits.length():
          print "Beam cannot be found. Check beam height and power."
          return None
        number_of_scans += 1
        scan_direction_x = -scan_direction_x
        if beam_position is None:
          if self.axes[1].position() == self.axes[1].limits.upper:
            self.axes[1].position(0, wait=True)
          else:
            self.axes[1].position(self.axes[1].position() + 3.0, wait=True)


    # Move back to beam position.
    self.change_grouping(1, fast=True)
    self.position(beam_position, wait=True)

    # The beam should now be in view.
    self.change_grouping(3, fast=True)
    beam_position = self.center_beam()
    if beam_position is None:
      # If it's not in view do a short scan to refind it.
      self.change_grouping(1, fast=False)
      self.position(
          self.position() + [
            self.facing_z_direction * scan_direction_x * 18.0, 0, 0],
          wait=False)
      beam_position = self._scan_until_beam_visible(x_axis)
      if beam_position is None:
        self.position(
            self.position() + [
              self.facing_z_direction * scan_direction_x * -36.0, 0, 0],
            wait=False)
        beam_position = self._scan_until_beam_visible(x_axis)
      beam_position = self.center_beam()
    # Return position or None, whatever it may be.
    return beam_position

  def find_beam_trajectory(self, start_point=None,
      scan_direction_x=1, scan_direction_z=1, z_samples=5):
    """Find trajectory of single beam.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.
      scan_direction_z (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) z direction.
      z_samples (5): Number of points to sample the beam.

    """
    beam = Beam()
    # Find the beam at the first z extreme.
    if start_point is None:
      start_point = Vector([
          self.axes[0].limits.direction(-scan_direction_x),
          self.axes[1].limits.middle(),
          self.axes[2].limits.direction(-scan_direction_z)])

    delta_z = -scan_direction_z * self.axes[2].limits.length() / z_samples
    sample_positions = list(arange(
            self.axes[2].limits.direction(scan_direction_z),
            self.axes[2].limits.direction(-scan_direction_z),
            delta_z))
    sample_positions.reverse()

    intercept = self.find_beam_center(start_point, scan_direction_x)
    if intercept is None:
      print "Trajectory cannot be established."
      return None

    # Calculate rough trajectory of the beam.
    beam.add_sample(intercept)
    self.change_grouping(1, fast=True)
    small_step = intercept + (scan_direction_z * array([0, 0, 10]))
    if abs(delta_z) > 10:
      self.position(small_step, wait=True)
    else:
      self.position(intercept + (
          scan_direction_z * array([0, 0, abs(delta_z)])), wait=True)
    intercept = self.center_beam()
    while intercept is None:
      small_step = small_step - (scan_direction_z * array([0, 0, 2]))
      self.position(small_step, wait=True)
      intercept = self.center_beam()
    beam.add_sample(intercept)
    if sample_positions[0] == intercept[2]:
      sample_positions.pop(0)
    for z_sample in sample_positions:
      self.position(beam.position(z_sample), wait=True)
      intercept = self.center_beam()
      if intercept is not None:
        beam.add_sample(intercept)
      else:
        # Cross this bridge when we get there.
        pass
    beam_angle = sys_math.degrees(sys_math.asin(beam.azimuth().value))
    stage_angle = self.devices['r_stage'].position()
    self.rotate(stage_angle - beam_angle, wait=True)
    self.center_beam()
    beam.power = (
        self.devices['profiler'].average_power(),
        self.devices['monitor'].read())
    self.rotate(stage_angle, wait=True)
    return beam

