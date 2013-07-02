# -*- coding: utf-8 -*-
"""
A class to read the data from a Newport HD-LBP laser beam profiler.

"""
import serial
import numpy as np
from collections import namedtuple
from itop.beam.beam import Beam
import datetime
import zlib
import cPickle

Alignment = namedtuple('Alignment',
    ['beam_a',
     'beam_b',
     'angles',
     'x_displacement',
     'y_displacement',
     'date'])

class Profiler(object):
  """Provides an interface to a Newport HD-LBP over serial link.

  """

  def __init__(self, device):
    """Establish serial communication with an HD-LBP.

    """
    self.device = device
    self.serial = serial.Serial(device, 115200, timeout=1)
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r',
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']

  def read(self):
    """Read the latest recorded data.

    The output is a dictionary that contains the following quantities:
      'time' - Time since camera reset of measurement (seconds)
      'power' - Power deposited on CCD (mW). Requires calibration for accuracy.
      'centroid_x' - Centroid horizontal position from center (micrometers)
      'centroid_y' - Centroid vertical position from center (micrometers)
      'centroid_r' - Image radius (micrometers)

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
      if ' \n' in readout:
        lines = readout.split(' \n')
        if len(lines) > 2:
          last_full_line = lines[-2]
          values = last_full_line.split(" ", 1)[1]
          floats = [float(x) for x in values.split()]
          if len(floats) == 14:
            output = dict(zip(self.keys, floats))
            return output


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
      'power' (0.003 mW): Power threshold when beam in view.

    """
    self.driver = driver
    self.rotation_stage = rotation_stage
    self.profiler = profiler
    self.alignment = None
    xyz_axes = kwargs.pop('xyz_axes', [1, 2, 3])
    self.axes = (self.driver.axes[xyz_axes[0] - 1],
                 self.driver.axes[xyz_axes[1] - 1],
                 self.driver.axes[xyz_axes[2] - 1])
    self.group_state = 3 # 1=axes independent, 2=xz grouped, 3=xyz grouped
    self.facing_z_direction = kwargs.pop('facing_z_direction', -1)

    # Optional instance variables.
    self.power = kwargs.pop('power', 0.003)
    self.group_id = kwargs.pop('group_id', 1)

    alignment_path = kwargs.pop('alignment_path', None)
    self.driver.group_create(self.axes, **kwargs)
    if alignment_path is not None:
      self.load_alignment(alignment_path)

  def axis(self, axis_id):
    """Returns the stage instance given by the axis_id (1,2,3)
    """

  def align(self):
    """Determines the alignment of the tracker with respect to
    the incoming beams.

    """
    beam_a = Beam(self)
    beam_b = Beam(self)
    # Rotate profiler to face splitter output.
    self.rotation_stage.power_on()
    self.rotation_stage.go_to_home(wait=True)
    self.rotation_stage.position(180, wait=True)
    self.facing_z_direction = 1
    shutter = self.driver.shutter_state
    shutter(0, 0)
    shutter(1, 1)
    beam_a.find_trajectory(
        [self.driver.a, 12, 125], -1, -1)
    # Block beam 'A' and find beam 'B' trajectory.
    shutter(1, 0)
    shutter(0, 1)
    beam_b.find_trajectory([125-50, 8, 125], -1, -1)
    x_displacement, y_displacement = (
        beam_b.upstream_point - beam_a.upstream_point)[:2]
    angles = [angle for angle in beam_a.angles()]
    self.alignment = Alignment(
        beam_a.trajectory(), beam_b.trajectory(),
        angles, x_displacement, y_displacement,
        datetime.datetime.utcnow().isoformat())
    # Rotate camera to face mirror.
    self.rotation_stage.position(0, wait=True)
    self.facing_z_direction = -1
    shutter(1, 1)

  def save_alignment(self, file_path):
    """Saves the beam alignment data to a gzipped serialized object file.

    """
    with open(file_path, 'wb') as output_file:
      output_file.write(zlib.compress(
          cPickle.dumps(self.alignment, cPickle.HIGHEST_PROTOCOL),9))

  def load_alignment(self, file_path):
    """Loads the beam alignment data from a gzipped serialized object file.

    """
    with open(file_path, 'rb') as input_file:
      pickled_data = zlib.decompress(input_file.read())
      self.alignment = cPickle.loads(pickled_data)

  def alignment_date(self):
    """Returns the date and time the current alignment data was taken.

    """
    if self.alignment is None:
      return 'invalid'
    else:
      return self.alignment.date

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
        position.insert(1, self.axes[1].position())
        return position
      else:
        self.driver.group_move_line(self.group_id, coords[0::2])
        self.axes[1].position(coords[1])
        if wait:
          self.driver.axis1.pause_for_stage()
          self.driver.axis2.pause_for_stage()
          self.driver.axis3.pause_for_stage()

    def ungrouped(coords):
      """Action to take if no stages are grouped."""
      if coords is None:
        return [self.axes[0].position(),
                self.axes[1].position(),
                self.axes[2].position()]
      else:
        self.axes[0].position(coords[0])
        self.axes[1].position(coords[1])
        self.axes[2].position(coords[2])
        if wait:
          self.driver.axis1.pause_for_stage()
          self.driver.axis2.pause_for_stage()
          self.driver.axis3.pause_for_stage()

    cases = {
        1: ungrouped,
        2: xz_grouped,
        3: xyz_grouped,
        }

    return cases[self.group_state](xyz_coordinates)

  def centroid(self):
    """If the beam is visible, returns the centroid xyz coordinates [mm] in the
    profiler coordinate system. The z coordinate is always 0, but included so
    the centroid position can be directly added to the stage coordinate system.

    If the beam is not in view, None is returned.

    """
    output = self.beam_visible()
    if output is None:
      return None
    else:
      return [-1 * self.facing_z_direction * output['centroid_x'] / 1000.0,
              output['centroid_y'] / 1000.0, 0.0]

  def beam_position(self):
    """If visible, returns the position of the beam centroid in the stage
    coordinate system. Otherwise returns None

    """
    centroid = self.centroid()
    if centroid is None:
      return None
    else:
      stage_coordinates = self.stage_position()
      return (np.array(stage_coordinates) + np.array(centroid)).tolist()

  def beam_visible(self):
    """Returns the beam profile if beam is in the frame, otherwise returns None.

    """
    profile = self.profiler.read()
    if profile['power'] >= self.power:
      return profile
    else:
      return None

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
    axis_ids = [axis.axis_id for axis in self.axes]
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
