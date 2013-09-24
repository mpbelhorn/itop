# -*- coding: utf-8 -*-
"""
A class to read the data from a Newport HD-LBP laser beam profiler.

"""
import serial
from numpy import mean, std
from itop.math import Value
from time import time as clock


class ProfilerError(EnvironmentError):
  """Profiler error exception handeler."""
  pass

class Profiler(object):
  """Provides an interface to a Newport HD-LBP over serial link.

  """

  def __init__(self, device, nominal_power=16.00):
    """Establish serial communication with an HD-LBP.

    """
    self.device = device
    self.serial = serial.Serial(device, 115200, timeout=1)
    self.serial.flushOutput()
    if not self.serial.read(10):
      print('No response from profiler. Check link.')
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r',
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']
    self.power_levels = self.set_power_levels(nominal_power)

  def set_power_levels(self, nominal_power):
    """Set the beam power levels given the nominal beam power.

    Two levels are set:
      2.0% nominal beam power - Used to find front face reflections.
      50.0% nominal beam power - Used to find main reflections.

    """
    self.power_levels = [
        0.50 * nominal_power,
        0.02 * nominal_power,
        ]
    return self.power_levels


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
    start_time = clock()
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
        if clock() - start_time > 5:
          raise ProfilerError(
              1, 'No response from profiler. Is it on and transmitting?')

  def profile(self, level=None):
    """Returns the beam profile if the beam is in view.

    Optional arguments:
      level=(n:None): Only return the profile if the visible beam power is in
          the half-open interval [level(n), level(n-1)), where level(n) is
          the nth entry of Profiler.power. If no level is given, the highest
          level power (lowest index) is used. If the level is out of range,
          the profile is returned if the visible beam power is greater than
          the lowest power P > level(n_max).
    """
    profile = self.read()
    power = profile['power']
    levels = self.power_levels
    if not level:
      if levels[0] <= power:
        return profile
    else:
      try:
        if (levels[level] <= power < levels[level - 1]):
          return profile
      except IndexError:
        if levels[-1] <= power:
          return profile
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

  def average_power(self, samples=10):
    """Return the average power of a number of samples with standard error."""
    try:
      data = [self.profile()['power'] for _ in range(samples)]
    except TypeError:
      return None
    return Value(mean(data), std(data))


