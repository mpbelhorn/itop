# -*- coding: utf-8 -*-
"""
A class to read the data from a Newport HD-LBP laser beam profiler.

"""
import serial
from numpy import array, mean, std, arange
from itop.beam import Beam
from itop.math import Vector
import datetime


class Alignment(object):
  """A class to establish the alignment between a tracker and the beams.

  """
  def __init__(self):
    """Constructor for alignment."""
    self.beam_a = None
    self.beam_b = None
    self.displacement = None  # r_b(x,y,0) - r_a(x,y,0) in tracker frame.
    self.date = None

  def align(self, tracker, home=False):
    """Establishes the alignment between the given tracker and the beams.

    Takes an optional keyword argument
    home (False): Recalibrates the rotation stage to it's home switch.

    """
    self.beam_a = Beam(-1)
    self.beam_b = Beam(-1)
    tracker.rotation_stage.power_on()
    if home:
      tracker.rotation_stage.go_to_home(wait=True)
    tracker.rotation_stage.position(180, wait=True)
    tracker.facing_z_direction = 1
    shutter = tracker.driver.shutter_state
    shutter(0, 0)
    shutter(1, 1)
    self.beam_a = tracker.find_beam_trajectory(
        [tracker.axes[0].limits.upper, 6, 125],
        -1, -1, z_samples=25)
    shutter(1, 0)
    shutter(0, 1)
    self.beam_b = tracker.find_beam_trajectory([125-50, 0, 125],
        -1, -1, z_samples=25)
    self.displacement = self.beam_b.intercept - self.beam_a.intercept
    self.date = datetime.datetime.utcnow().isoformat()

    # Rotate camera to face mirror.
    tracker.rotation_stage.position(0, wait=True)
    tracker.facing_z_direction = -1
    shutter(1, 1)

  def alignment_date(self):
    """Returns the date and time the current alignment data was taken.

    """
    if self.date is None:
      return 'invalid'
    else:
      return self.date



class Profiler(object):
  """Provides an interface to a Newport HD-LBP over serial link.

  """

  def __init__(self, device, threshold_power=0.003):
    """Establish serial communication with an HD-LBP.

    """
    self.device = device
    self.serial = serial.Serial(device, 115200, timeout=1)
    if not self.serial.read(10):
      raise serial.SerialException("Profiler data cannot be read.")
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r',
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']
    self.power = threshold_power

  def read(self):
    """Read the latest recorded data.

    The output is a dictionary that contains the following quantities:
      'time' - Time since camera reset of measurement (seconds)
      'power' - Power deposited on CCD (mW). Requires calibration for accuracy.
      'centroid_x' - Centroid horizontal position from center (millimeters)
      'centroid_y' - Centroid vertical position from center (millimeters)
      'centroid_r' - Image radius (millimeters)

      The following sizes of the x-,y-projected image are also given. See
      HD-LBP documentation for more information.
      'level_1' - Projection level 1 (13.5%)
      'level_2' - Projection level 2 (50.0 %)
      'level_3' - Projection level 3 (80.0%)
      'width_1' - Projection width at level 1
      'width_2' - Projection width at level 2
      'width_3' - Projection width at level 3
      'height_1' - Projection height at level 1
      'height_2' - Projection height at level 1
      'height_3' - Projection height at level 1

    """
    # Clear the current contents of the read buffer.
    self.serial.flushInput()
    readout = ''
    while True:
      readout = readout + self.serial.read(self.serial.inWaiting())
      try:
        # ValueError raised if '$R' and '\n' not in readout at least once.
        # single_line is '' if $R not ahead of \n and separated by characters.
        single_line = readout[readout.index('$R'):readout.rindex('\n')]
        # IndexError raised if single_line is ''.
        data = [float(i) for i in single_line.split('$R')[1].strip().split()]
        data[1] /= 1000.0
        data[2] /= 1000.0
        data[3] /= 1000.0
        return dict(zip(self.keys, data))
      except (ValueError, IndexError):
        pass

  def profile(self):
    """Returns the beam profile if the beam is in view."""
    profile = self.read()
    if profile['power'] >= self.power:
      return profile
    else:
      return None

  def distortion(self):
    """Returns a list of the ratios r(%) = h(%)/w(%) where h(%) and w(%) are the
    width and height of the beam's best fit gaussian profile at the given
    percentage of the maximum profile power.

    The default percentages are 13.5%, 50.0% and 80.0%

    """
    profile = self.read()
    return (profile['height_1']/profile['width_1'],
            profile['height_2']/profile['width_2'],
            profile['height_3']/profile['width_3'])


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

  def __init__(self, driver, rotation_stage, profiler, **kwargs):
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
    self.driver = driver
    self.rotation_stage = rotation_stage
    self.profiler = profiler
    xyz_axes = kwargs.pop('xyz_axes', [1, 2, 3])
    self.axes = (self.driver.axes[xyz_axes[0] - 1],
                 self.driver.axes[xyz_axes[1] - 1],
                 self.driver.axes[xyz_axes[2] - 1])
    self.group_state = 1 # 1=axes independent, 2=xz grouped, 3=xyz grouped
    self.facing_z_direction = kwargs.pop('facing_z_direction', -1)

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
      self.driver.group_create(axis_ids, **kwargs)
      self.group_state = 3
    elif state == 2:
      kwargs = ils_configuration
      self.driver.group_create(axis_ids[0::2], **kwargs)
      self.group_state = 2
    else:
      self.driver.group_delete(self.group_id)
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

  def stage_position(self, xyz_coordinates=None, wait=False):
    """Returns the stage position of the stage. If passed a set of coordinates,
    also moves stage to that position.

    """
    def xyz_grouped(coords):
      """Action to take if all 3 stages are grouped."""
      if coords is None:
        return self.driver.group_position(self.group_id)
      else:
        self.driver.group_move_line(self.group_id, coords, wait=wait)

    def xz_grouped(coords):
      """Action to take if only x-z stages are grouped."""
      if coords is None:
        position = self.driver.group_position(self.group_id)
        position = position.project([0, None, 1])
        position[1] = self.axes[1].position()
        return position
      else:
        self.driver.group_move_line(self.group_id, coords[0::2])
        self.axes[1].position(coords[1])
        if wait:
          self.driver.pause_for_stages()

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
          self.driver.pause_for_stages()

    cases = {
        1: ungrouped,
        2: xz_grouped,
        3: xyz_grouped,
        }

    return cases[self.group_state](xyz_coordinates)


  def centroid(self, samples=1):
    """If the beam is visible, returns the centroid xyz coordinates [mm] in the
    profiler coordinate system. The z coordinate is always 0, but included so
    the centroid position can be directly added to the stage coordinate system.

    If the beam is not in view, None is returned.

    """
    profiles = []
    for _ in range(samples):
      profile = self.profiler.profile()
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
    centroid = self.centroid(samples)
    if centroid is None:
      return None
    else:
      return self.stage_position() + centroid

  def center_beam(self):
    """Centers the camera on the beam if beam is visible. If the beam is already
    centered, the function returns the position of the beam in the relative
    to the stage group home + uncertainty. If the beam is visible and not
    centered, the camera is moved to the center. If the beam is not visible,
    None is returned.

    """
    centroid = self.centroid()
    if centroid is None:
      return None
    # Do quick sampling to get centroid close to center.
    while centroid != Vector([0.0, 0.0, 0.0], 0.050):
      rough_position = self.stage_position() + centroid
      self.stage_position(rough_position, wait=True)
      centroid = self.centroid()
    # Perhaps replace following while loop with a finite number of iterations?
    while True:
      centroid = self.centroid(8)
      centered_position = centroid + self.stage_position()
      if centroid != Vector([0.0, 0.0, 0.0], 0.001):
        self.stage_position(centered_position, wait=True)
      else:
        return centered_position

  def find_beam_center(self, start_point=None, scan_direction_x=1):
    """Scans in X for a single beam and centers it on the CCD.

    The optional arguments are:
      scan_direction_x (1): Integers +1 (-1) indicate to scan in the
                            positive (negative) x direction.

    """
    if start_point is None:
      start_point = [
          self.axes[0].limits.lower,
          self.axes[1].limits.middle(),
          self.axes[2].limits.lower]
    x_axis = self.axes[0]
    beam_position = self.get_beam_position()
    self.change_grouping(1, fast=True)
    if beam_position is None:
      # Move camera into starting point.
      self.stage_position(start_point)
      while x_axis.is_moving():
        beam_position = self.get_beam_position()
        if beam_position is not None:
          x_axis.stop(wait=True)
          break

    if beam_position is not None:
      beam_position = Vector(
          [beam_position[0], beam_position[1], start_point[2]])
      self.stage_position(beam_position, wait=True)

    # Scan for beam crossing if beam wasn't seen moving to start point.
    if beam_position is None:
      self.change_grouping(1, fast=False)
      x_axis.position(self.axes[0].limits.direction(scan_direction_x))
      while x_axis.is_moving():
        beam_position = self.get_beam_position()
        if beam_position is not None:
          x_axis.stop(wait=True)
          break
      else:
        print "Beam not seen!"
        return None

    self.change_grouping(1, fast=True)
    # Move back to beam position.
    self.stage_position(beam_position, wait=True)

    # The beam should now be in view.
    self.change_grouping(3, fast=True)
    while True:
      centered_position = self.center_beam()
      if centered_position is None:
        self.stage_position(
            self.stage_position() - [scan_direction_x * 2.0, 0, 0], wait=True)
      else:
        return centered_position

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

    sample_positions = list(arange(
            self.axes[2].limits.direction(scan_direction_z),
            self.axes[2].limits.direction(-scan_direction_z),
            -scan_direction_z * self.axes[2].limits.length() / z_samples))
    sample_positions.reverse()

    intercept = self.find_beam_center(start_point, scan_direction_x)
    for i in [1, -1, 2, -2]:
      if intercept is None:
        intercept = self.find_beam_center(
            start_point + [0, i * 4.0, 0], scan_direction_x)
      else:
        break
    else:
      print "Cannot find beam. Check beam power and camera height."
      return None

    # Calculate rough trajectory of the beam.
    beam.add_sample(intercept)

    delta_z = scan_direction_z * array([0, 0, 10])

    small_step = intercept + delta_z
    # TODO - Optimize stage speeds here to minimize vibration.
    self.change_grouping(1, fast=True)
    self.stage_position(small_step, wait=True)
    intercept = self.center_beam()
    while intercept is None:
      small_step = small_step - scan_direction_z * array([0, 0, 10])
      self.stage_position(small_step, wait=True)
      intercept = self.center_beam()
    beam.add_sample(intercept)
    for z_sample in sample_positions:
      self.stage_position(beam.position(z_sample), wait=True)
      intercept = self.center_beam()
      if intercept is not None:
        beam.add_sample(intercept)
      else:
        # Cross this bridge when we get there.
        pass

    return beam
