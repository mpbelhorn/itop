"""
A class to read the data from a Newport HD-LBP laser beam profiler.
"""
import serial
import numpy as np

class Profiler(object):
  """
  Provides an interface to a Newport HD-LBP over serial link.
  """

  def __init__(self, device):
    """
    Establish serial communication with an HD-LBP.
    """
    self.device = device
    self.io = serial.Serial(device, 57600, timeout=1)
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r',
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']

  def read(self):
    """
    Read the latest recorded data.

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
    self.io.flushInput()
    readout = ''
    while True:
      readout = readout + self.io.read(self.io.inWaiting())
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
  """
  A class to represent an HD-LBP on a set of ESP stages.
  """
  # Static configurations dictionary.
  configurations = {
      'Fast ILS': {'velocity': 40.0, 'acceleration': 20, 'deceleration': 20},
      'Slow ILS': {'velocity': 10.0, 'acceleration': 20, 'deceleration': 20},
      'Fast LTA': {'velocity':  5.0, 'acceleration': 20, 'deceleration': 20},
      'Slow LTA': {'velocity':  2.0, 'acceleration': 20, 'deceleration': 20},
      }

  def __init__(self, driver, profiler, xyz_axes=[1, 2, 3], **kwargs):
    """
    Creates a beam tracker.

    Optional keyword arguments:
    'power' (0.003 mW): LBP
    """
    self.driver = driver
    self.profiler = profiler
    self.axes = xyz_axes
    self.group_state = 3 # 1=axes independent, 2=xz grouped, 3=xyz grouped

    # Optional instance variables.
    self.power = kwargs.pop('power', 0.003)
    self.group_id = kwargs.get('group_id', 1)
    self.driver.groupCreate(self.axes, **kwargs)

  def stagePosition(self, xyz_coordinates=None, wait=False):
    """
    Returns the stage position of the stage. If passed a set of coordinates,
    also moves stage to that position.
    """
    def xyzGrouped(coords):
      if coords is None:
        return self.driver.groupPosition(self.group_id)
      else:
        self.driver.groupMoveLine(self.group_id, coords, wait=wait)

    def xzGrouped(coords):
      if coords is None:
        position = self.driver.groupPosition(self.group_id)
        position.insert(1, self.driver.axes[self.axes[1] - 1].position())
        return position
      else:
        self.driver.groupMoveLine(self.group_id, coords[0::2])
        self.driver.axes[self.axes[1] - 1].position(coords[1])
        if wait:
          self.driver.axis1.pauseForStage()
          self.driver.axis2.pauseForStage()
          self.driver.axis3.pauseForStage()

    def ungrouped(coords):
      if coords is None:
        return [self.driver.axes[self.axes[0] - 1].position(),
                self.driver.axes[self.axes[1] - 1].position(),
                self.driver.axes[self.axes[2] - 1].position()]
      else:
        self.driver.axes[self.axes[0] - 1].position(coords[0])
        self.driver.axes[self.axes[1] - 1].position(coords[1])
        self.driver.axes[self.axes[2] - 1].position(coords[2])
        if wait:
          self.driver.axis1.pauseForStage()
          self.driver.axis2.pauseForStage()
          self.driver.axis3.pauseForStage()

    cases = {
        1: ungrouped,
        2: xzGrouped,
        3: xyzGrouped,
        }

    return cases[self.group_state](xyz_coordinates)

  def centroid(self):
    """
    If the beam is visible, returns the centroid xyz coordinates [mm] in the
    profiler coordinate system. The z coordinate is always 0, but included so
    the centroid position can be directly added to the stage coordinate system.

    If the beam is not in view, None is returned.
    """
    output = self.beamVisible()
    if output is None:
      return None
    else:
      return [output['centroid_x'] / 1000.0, output['centroid_y'] / 1000.0, 0.0]

  def beamPosition(self):
    """
    If visible, returns the position of the beam centroid in the stage
    coordinate system. Otherwise returns None
    """
    centroid = self.centroid()
    if centroid is None:
      return None
    else:
      stage_coordinates = self.stagePosition()
      return (np.array(stage_coordinates) + np.array(centroid)).tolist()

  def beamVisible(self):
    """
    Returns the beam profile if beam is in the frame, otherwise returns None.
    """
    profile = self.profiler.read()
    if profile['power'] >= self.power:
      return profile
    else:
      return None

  def groupState(self, state=None, fast=False):
    """
    Groups the tracker stages into one of 3 modes given by the 'state'
    argument. If no state is given, function returns the current state.

    Possible states are:
    1: Ungrouped   - Stages are ungrouped and can be moved independently.
    2: XZ Grouped  - X and Z axes are grouped together, Y is independent.
    3: XYZ Grouped - All stages are grouped together. Group kinematics are
                     limited by the slowest stage in the group.

    This function takes the same keyword arguments as
    'itop.motioncontrol.controller.groupCreate'.
    """
    if state is None:
      return self.group_state
    else:
      if state == 3:
        kwargs = Tracker.configurations['Fast LTA' if fast else 'Slow LTA']
        self.driver.groupCreate(self.axes, **kwargs)
        self.group_state = 3
      elif state == 2:
        kwargs = Tracker.configurations['Fast ILS' if fast else 'Slow ILS']
        self.driver.groupCreate(self.axes[0::2], **kwargs)
        self.group_state = 2
      else:
        self.driver.groupDelete(self.group_id)
        self.driver.axes[self.axes[0] - 1].velocity(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['velocity'])
        self.driver.axes[self.axes[0] - 1].acceleration(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['acceleration'])
        self.driver.axes[self.axes[0] - 1].deceleration(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['deceleration'])
        self.driver.axes[self.axes[1] - 1].velocity(
            Tracker.configurations['Fast LTA' if fast else 'Slow LTA']['velocity'])
        self.driver.axes[self.axes[1] - 1].acceleration(
            Tracker.configurations['Fast LTA' if fast else 'Slow LTA']['acceleration'])
        self.driver.axes[self.axes[1] - 1].deceleration(
            Tracker.configurations['Fast LTA' if fast else 'Slow LTA']['deceleration'])
        self.driver.axes[self.axes[2] - 1].velocity(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['velocity'])
        self.driver.axes[self.axes[2] - 1].acceleration(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['acceleration'])
        self.driver.axes[self.axes[2] - 1].deceleration(
            Tracker.configurations['Fast ILS' if fast else 'Slow ILS']['deceleration'])
        self.group_state = 1
