# -*- coding: utf-8 -*-
"""
This module provides methods for controlling stages attached to an EPS300
motion controller.

"""
from itop.utilities import clamp
from itop.math import Value

class Limits:
  """A class to represent the limits of motion of a stage.

  """
  def __init__(self, limit_1=0, limit_2=0):
    """Constructor for Limits.

    """
    self.lower = float(min(limit_1, limit_2))
    self.upper = float(max(limit_1, limit_2))

  def middle(self):
    """Returns the center of the limit range.

    """
    return (self.upper + self.lower) / 2.0

  def direction(self, direction):
    """Returns the limit in the given direction, where direction can be either
    1 or -1.

    """
    return self.upper if direction > 0 else self.lower

  def length(self):
    """Returns the length of the stage."""
    return self.upper - self.lower


class Stage(object):
  """A represenation of a robotic stage.

  The methods allow for the control and monitoring of specific servo or
  stepper motor driven robotic stages through a Newport ESP30X stage controller.

  """
  def __init__(self, axis, controller, limits=None):
    """Constructor for Stage.

    Requires a controller instance and an axis number on that controller.

    Optionally, stage motion limits can be defined at construction by
    passing either an iterable of the form (lower_limit, upper_limit) or
    passing a single limit where the stage can travel between ±limit.

    """
    self.axis_id = str(axis)
    self.controller = controller
    self.resolution = 0.0001 # TODO - Read this from the stage.
    self.limits = Limits(0, 0)
    if limits is not None:
      try:
        self.limits = Limits(*sorted(limits))
      except TypeError:
        self.limits = Limits(-1 * abs(limits), abs(limits))

  def set_limits(self, limits=None):
    """Defines the stage limits.

    Takes either an iterable in the form (lower_limit, upper_limit) or a
    single value where the stage can travel between ±value.

    """
    self.limits = Limits(0, 0)
    if limits is not None:
      try:
        self.limits = Limits(*sorted(limits))
      except TypeError:
        self.limits = Limits(-1* abs(limits), abs(limits))

  def send(self, command, parameter=''):
    """Send a command to this axis.

    """
    self.controller.send(command, str(parameter), self.axis_id)

  def targeted_position(self):
    """Returns the position the stage is currently targeting.

    """
    self.send('DP?')
    position = self.controller.read()
    print position, self.units()
    return float(position)

  def targeted_velocity(self):
    """Returns the stage's targeted velocity.

    """
    self.send('DV')
    velocity = self.controller.read()
    print velocity+self.units()+'/s'
    return float(velocity)

  def stage_id(self):
    """Returns stage model and serial number.

    """
    self.send('ID')
    return self.controller.read()

  def is_moving(self):
    """Return false for stopped, true for in motion.

    """
    self.send('MD?')
    return True if '0' in self.controller.read() else False

  def power_on(self):
    """Turns the axis motor on.

    """
    self.send('MO')

  def power_off(self):
    """Turns the axis motor off.

    """
    self.send('MF')

  def define_home(self, position=None):
    """Sets the stage home position to given position in current units.

    """
    if position is None:
      self.send('DH', '?')
      position = self.controller.read()
    else:
      if self.limits is not None:
        self.limits = Limits([i + position for i in self.limits])
      self.send('DH', position)
    return float(position)

  def move_to_limit(self, direction=None):
    """Given the argument '+' or '-', moves stage that hardware limit.

    """
    if direction is None:
      self.send('MT', '?')
      finished = self.controller.read()
    else:
      self.send('MT', direction)
    return int(finished)

  def move_indefinately(self, direction=None):
    """Initiates continuous motion in the given '+' or '-' direction.

    If no argument is given, command reports when it has hit a limit.

    """
    if direction is None:
      self.send('MV', '?')
      finished = self.controller.read()
      return int(finished)
    else:
      self.send('MV', direction)

  def move_to_next_index(self, direction=None):
    """Moves to the nearest index in the given '+' or '-' direction.

    """
    finished = 0
    if direction is None:
      self.send('MZ', '?')
      finished = self.controller.read()
    else:
      self.send('MZ', direction)
    return int(finished)

  def go_to_home(self, **kwargs):
    """Moves the stage to the home position.

    """
    self.send('OR')
    if kwargs.pop('wait', False):
      self.pause_for_stage()

  def position(self, position=None, wait=False):
    """Moves the stage to an absolute position. If no argument is
    given, the current position of the stage is returned.

    If the given position is outside the range of the stage, the
    stage is moved to its limit.
    """
    if position is None:
      self.send('TP')
      position = float(self.controller.read())
    else:
      if self.limits is not None:
        position = clamp(position, self.limits.lower, self.limits.upper)
      self.send('PA', position)
    if wait:
      self.pause_for_stage()
    return Value(position, self.resolution)

  def move(self, relative_position, wait=False):
    """Moves the stage the given relative position.

    """
    requested_position = self.position() + relative_position
    self.position(requested_position, wait)
    return float(relative_position)

  def pause_for_stage(self):
    """Hold python execution in null loop until stage is stopped.

    """
    while self.is_moving():
      pass

  def stop(self, **kwargs):
    """Stops motion on this axis with predefined acceleration.

    """
    self.send('ST')
    if kwargs.pop('wait', False):
      self.pause_for_stage()

  def step_resolution(self, resolution=None):
    """Sets or returns the encoder full-step resolution for a Newport Unidrive
    compatible programmable driver with step motor axis.

    """
    if resolution is None:
      self.send('FR', '?')
      resolution = self.controller.read()
    else:
      self.send('FR', resolution)
    return float(resolution)

  def gear_ratio(self, gear_ratio=None):
    """Sets or returns the master-slave reduction ratio for a slave axis.

    Use this command very carefully. The slave axis will have its speed and
    acceleration in the same ratio as the position.
    Also, ensure that the ratio used for the slave axis does not cause
    overflow of this axis parameters (speed, acceleration), especially with
    ratios greater than 1.

    """
    if gear_ratio is None:
      self.send('GR', '?')
      gear_ratio = self.controller.read()
    else:
      self.send('GR', gear_ratio)
    return float(gear_ratio)

  def units(self, units=None):
    """Sets the stage displacement units from given integer.
    If no argument is given, current unit setting is reported.

    Possible units:
    0 -- Encoder counts         6 -- micro-inches
    1 -- Motor steps            7 -- degrees
    2 -- millimeters            8 -- gradient
    3 -- micrometers            9 -- radians
    4 -- inches                10 -- milliradian
    5 -- mils (milli-inches)   11 -- microradian

    """
    if units is None:
      self.send('SN', '?')
      units = int((self.controller.read()).strip())
    else:
      self.send('SN', units)
    unit_codes = [
        'encoder-counts', 'motor-steps', 'mm', u'\u03BCm', 'in','mil',
         u'\u03BCin', u'\u00B0', 'grade', 'rad', 'mrad', u'\u03BCrad']
    return unit_codes[units]

  def following_error_threshold(self, error=None):
    """Sets or returns the maximum allowed following error.

    """
    if error is None:
      self.send('FE', '?')
      error = self.controller.read()
    else:
      self.send('FE', error)
    return float(error)

  def following_error_configuration(self, configuration=None):
    """Sets the stage response when following error is exceeded.

    The configuration is a hex string. The string must start
    with a zero, ie 0F. Do not include encoding prefix (i.e. 0x0F).

    The configuration must have the binary equivalent
    '0000 0xyz' where a 1(0) on the bits x,y,z indicates:
    z = (Do not) Enable checking of following error.
    y = (Do not) Power off stage on excess following error
    x = (Do not) Abort motion on excess following error.

    Common values are
    0x03 0b0000011 default
    0x05 0b0000101 Abort motion on following error

    """
    if configuration is None:
      self.send('ZF', '?')
      configuration = self.controller.read()
    else:
      self.send('ZF', configuration)
    return configuration

  def acceleration(self, acceleration=None):
    """Sets the stage acceleration.

    """
    if acceleration is None:
      self.send('AC', '?')
      acceleration = self.controller.read()
    else:
      self.send('AC', acceleration)
    return float(acceleration)

  def e_stop_acceleration(self, acceleration=None):
    """Sets the stage emergency stop acceleration.

    """
    if acceleration is None:
      self.send('AE', '?')
      acceleration = self.controller.read()
    else:
      self.send('AE', acceleration)
    return float(acceleration)

  def deceleration(self, deceleration=None):
    """Sets te stage deceleration.

    """
    if deceleration is None:
      self.send('AG', '?')
      deceleration = self.controller.read()
    else:
      self.send('AG', deceleration)
    return float(deceleration)

  def acceleration_limit(self, acceleration=None):
    """Sets the maximum allowed stage acceleration/deceleration.

    Stage will error out if this limit is exceeded.

    """
    if acceleration is None:
      self.send('AU', '?')
      acceleration = self.controller.read()
    else:
      self.send('AU', acceleration)
    return float(acceleration)

  def backlash_compensation(self, compensation=None):
    """Set or report the backlash compensation in current units.

    Maximum compensation is equivelent of 10000 encoder counts.

    """
    if compensation is None:
      self.send('BA', '?')
      compensation = self.controller.read()
    else:
      self.send('BA', compensation)
    return float(compensation)

  def home_preset(self, home_position=None):
    """Sets the absolute position ascribed to the home position.

    """
    if home_position is None:
      self.send('SH', '?')
      home_position = self.controller.read()
    else:
      if self.limits is not None:
        self.limits = Limits([i + home_position for i in self.limits])
      self.send('SH', home_position)
    return float(home_position)

  def velocity(self, velocity=None):
    """Sets the stage velocity.

    """
    if velocity is None:
      self.send('VA', '?')
      velocity = self.controller.read()
    else:
      self.send('VA', velocity)
    return float(velocity)

  def velocity_limit(self, velocity=None):
    """Sets the maximum allowed stage velocity.

    Stage will error out if this limit is exceeded.

    """
    if velocity is None:
      self.send('VU', '?')
      velocity = self.controller.read()
    else:
      self.send('VU', velocity)
    return float(velocity)

  def wait_until_position(self, position):
    """Pause EPS command execution until stage is at position.

    This does not pause execution of python code!

    """
    self.send('WP', position)

  def wait_until_stopped(self, time=''):
    """Pause EPS command execution time [ms] after stage is stopped.

    This does not pause execution of python code!

    """
    self.send('WS', time)


