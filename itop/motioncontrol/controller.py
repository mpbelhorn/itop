"""
This module provides methods for controlling and communicating with an EPS30x
motion controller.
"""

import serial
import itop.motioncontrol.stage as stage
import time

class StageController(object):
  """
  Encompasses serial I/O for Newport EPS300 motion controllers, controller
  operation methods plus instances of the stage I/O classes for each
  axis attached to the controller.
  """

  def __init__(self, serial_device):
    """
    Creates an I/O handle on a Newport EPS300 motion controller and its axes.

    Argumentx:
    serial_device -- Path string to serial port used by the controller.
    """
    self.io = serial.Serial(serial_device, 19200, timeout = 1)
    self.io_end = '\r\n'
    self.axis1 = stage.Stage(1, self)
    self.axis2 = stage.Stage(2, self)
    self.axis3 = stage.Stage(3, self)
    self.axes = [self.axis1, self.axis2, self.axis3]
    self.gpioDirections(0b01)
    self.readFirmwareVersion()

  def send(self, command, parameter = '', axis = ''):
    """
    Send a command to the controller.
    """
    self.io.write(str(axis) + str(command) + str(parameter) + self.io_end)

  def read(self):
    """
    Return a line read from the controller's serial buffer.
    """
    return self.io.readline()

  def reset(self):
    """
    Perform a full controller reset.
    """
    self.send('RS')

  def readStatus(self):
    """
    Read the status buffer.
    """
    self.send('TS')
    return self.read()

  def readActivity(self):
    """
    Read the activity register.
    """
    self.send('TX')
    return self.read()

  def error(self):
    """
    Read the first error message in the error FIFO buffer.
    """
    self.send('TB?')
    return self.read()

  def errors(self):
    """
    Read all the error messages in the error FIFO buffer.
    """
    messages = []
    self.send('TB?')
    messages.append(self.read())
    while messages[-1].split(',')[0] != '0':
      self.send('TB?')
      messages.append(self.read())
    return messages

  def readFirmwareVersion(self):
    """
    Report the controller firmware version.
    """
    self.send('VE?')
    return self.read()

  def wait(self, milliseconds='0'):
    """
    Pause internal command execution for given time.
    """
    self.send('WT', milliseconds)

  def eStop(self):
    """
    Emergency stop all axes.

    The e-stop configuration for each axis is invoked when this command is sent.
    """
    self.send('AB')

  def abortProgram(self):
    """
    Abort execution of current program.

    Stages in motion will finish their last command.
    """
    self.send('AB')

  def groupAcceleration(self, group_id, acceleration = '?'):
    """
    Sets the vectorial acceleration for a group.

    This command overides individually set accelerations.
    """
    self.send('HA', acceleration, group_id)
    if (acceleration == '?'):
      acceleration = float(self.read().strip())
    return acceleration

  def groups(self):
    """
    Returns the IDs of all defined groups.
    """
    self.send('HB')
    group_ids = self.read()
    return group_ids

  def groupMoveArc(self, group_id, coordinates = '?'):
    """
    Moves a group along an arc.

    The arc is defined by a list of coordinates:
      [center_x, center_y, deltaTheta]
    """
    if (coordinates == '?'):
      self.send('HC', coordinates, group_id)
      coordinates = self.read()
    else:
      self.send('HC', ",".join(map(str, coordinates)), group_id)
    return coordinates

  def groupDeceleration(self, group_id, deceleration = '?'):
    """
    Sets the vectorial deceleration for a group.

    This command overides individually set decelerations.
    """
    self.send('HD', deceleration, group_id)
    if (deceleration == '?'):
      deceleration = float(self.read().strip())
    return deceleration

  def groupEStopDeceleration(self, group_id, deceleration = '?'):
    """
    Sets the vectorial deceleration for a group emergency stop.

    This command overides individually set decelerations.
    """
    self.send('HE', deceleration, group_id)
    if (deceleration == '?'):
      deceleration = float(self.read().strip())
    return deceleration

  def groupOff(self, group_id):
    """
    Turns off power to all axis in a group.
    """
    self.send('HF', '', group_id)

  def groupJerk(self, group_id, jerk = '?'):
    """
    Sets the vectorial jerk limit for a group.

    This command overides individually set jerks.
    """
    self.send('HJ', jerk, group_id)
    if (jerk == '?'):
      jerk = float(self.read().strip())
    return jerk

  def groupMoveLine(self, group_id, coordinates = '?', **kwargs):
    """
    Moves a group along a line.

    The line is defined by a list of endpoint coordinates:
      [axis1, axis2, ..., axisN]

    Takes the optional keyword boolean wait to pause python execution
      for group to come to a stop. Default is wait=False.
    """
    if (coordinates == '?'):
      self.send('HL', coordinates, group_id)
      coordinates = self.read()
    else:
      self.send('HL', ",".join(map(str, coordinates)), group_id)
    if kwargs.pop('wait', False):
      self.pauseForGroup(group_id)
    return coordinates

  def groupCreate(self, axes=None, group_id=1, home=False,
      velocity=10, acceleration=30, deceleration=30, jerk=0, estop=200):
    """
    Creates a group of two or more axes over the given axes.

    If no axes are given, command returns the axes assigned to the group with
    ID=1 or, if supplied, the given optional group ID (see keyword arguments).

    The defined groups on the controller must have contiguous integer IDs from
    1 to MAX_GROUPS where

        MAX_GROUPS = floor(AXES_ON_CONTROLLER/2)

    and MAX_AXES is the number of axes on the controller.

    The axes are given as an ordered list of the physical axis numbers:
      [axis1, axis2, ..., axisN]

    Keyword arguments:
      'group_id' (1):
          The group ID to set/overwrite.
      'home' (False):
          Move axes to their home positions and reset the stage
          coordinate system.
      'velocity' (10|V_MAX):
          Set group velocity (Units/s). Defaults to 10 units/s or maximum
          allowed velocity if V_MAX < 10.
      'acceleration' (30|A_MAX):
          Set group acceleration (Units/s^2). Defaults to 100 units/s^2 or
          maximum allowed acceleration if A_MAX < 100.
      'deceleration' (30|D_MAX):
          Set group deceleration (Units/s^s). Same limits as for acceleration.
      'jerk' (0):
          Set group jerk rate (Units/s^3). Defaults to trapezoid motion
          profile. If jerk != 0, the group will move in an S-curve profile
          which is harder to stop/change motion characteristics despite
          being smoother.
      'estop' (200):
          Set group emergency stop deceleration (Units/s^2). Maximum allowed
          value is 2 * 10^9 * MIN_ENCODER_RESOLUTION.
    """
    group_id = kwargs.pop('group_id', 1)
    if axes is None:
      self.send('HN', '?', group_id)
      axes = [int(axis.strip()) for axis in self.read().split(',')]
    else:
      # NOTE - Deleting a lower ID group when more than one group exists
      #   is untested.
      self.groupDelete(group_id)
      kinematics = {
          'velocity': velocity,
          'acceleration': acceleration,
          'deceleration': deceleration,
          }
      for axis in axes:
        stage = self.axes[axis - 1]
        kinematics['velocity'] = min(
            kinematics['velocity'], stage.velocityLimit())
        kinematics['acceleration'] = min(
            kinematics['acceleration'], stage.accelerationLimit())
        kinematics['deceleration'] = min(
            kinematics['deceleration'], stage.accelerationLimit())
        if home:
          stage.on()
          stage.goToHome(wait=True)
      self.send('HN', ",".join(map(str, axes)), group_id)
      self.groupVelocity(group_id, kinematics['velocity'])
      self.groupAcceleration(group_id, kinematics['acceleration'])
      self.groupDeceleration(group_id, kinematics['deceleration'])
      self.groupJerk(group_id, jerk)
      self.groupEStopDeceleration(group_id, estop)
      self.groupOn(group_id)
    return axes

  def groupOn(self, group_id):
    """
    Turns on power to all axis in a group.
    """
    self.send('HO', '', group_id)

  def groupConfiguration(self, group_id):
    """
    Returns a dictionary of the configuration parameters
    needed to recreate the group using groupCreate()
    """
    configuration = dict()
    if str(group_id) in self.groups():
      configuration = dict([
          ('group_id', group_id),
          ('axes', self.groupCreate(group_id)),
          ('velocity', self.groupVelocity(group_id)),
          ('acceleration', self.groupAcceleration(group_id)),
          ('deceleration', self.groupDeceleration(group_id)),
          ('jerk', self.groupJerk(group_id)),
          ('estop', self.groupEStopDeceleration(group_id))])
    return configuration

  def groupPosition(self, group_id):
    """
    Returns the position of all stages in a group.

    The positions are given as an ordered list of the group axis numbers:
      [axis1, axis2, ..., axisN]
    """
    self.send('HP', '', group_id)
    coordinates = [float(x.strip()) for x in self.read().split(',')]
    return coordinates

  # Wait for group via point buffer. - NOT IMPLEMENTED.

  def groupStop(self, group_id):
    """
    Stops all motion on all axes in a group.
    """
    self.send('HS', '', group_id)

  def groupIsMoving(self, group_id):
    """
    Queries controller if group is in motion or stopped.
    """
    self.send('HS', '?', group_id)
    response = self.read()
    if '0' in response:
      return True
    else:
      return False

  def pauseForGroup(self, group_id):
    """
    Holds python execution in loop until group is stopped.
    """
    while self.groupIsMoving(group_id):
      pass

  def groupVelocity(self, group_id, velocity = '?'):
    """
    Sets the vectorial velocity limit for a group.

    This command overides individually set velocities.
    """
    self.send('HV', velocity, group_id)
    if (velocity == '?'):
      velocity = float(self.read().strip())
    return velocity

  def groupWaitForStop(self, group_id, delay = '0'):
    """
    Pauses command execution until group has stopped for given delay [ms].
    """
    self.send('HW', delay, group_id)

  def groupDelete(self, group_id):
    """
    Deletes group with given ID.
    """
    self.send('HX', '', group_id)

  def groupSize(self, group_id):
    """
    Returns the size of the group with given group_id.
    """
    self.send('HZ', '', group_id)
    size = self.read()
    return size

  def gpioDirections(self, directions=None):
    """
    Sets the given GPIO ports directions as input or output. If no input is
    give, returns the current setting.

    The directions are given as a binary number with each bit representing one
    of two GPIO ports (A,B), either can be set to 1 (output) or 0 (input).
    """
    if directions is None:
      self.send('BO', '?')
      return int(self.read().strip().replace('H', ''), 16)
    else:
      self.send('BO', hex(directions).replace('x', '') + 'H')

  def gpioState(self, status=None):
    """
    Sets the state of the GPIO pins on both ports (A,B) according to
    a given 16 bit binary input 'status'. If no input is given, returns
    the current status.

    Bit 0-7 (8-15) correspond to port A (B).
    """
    if status is None:
      self.send('SB','?')
      return int(self.read().strip().replace('H', ''), 16)
    else:
      self.send('SB', hex(status).replace('x', '') + 'H')

  def shutterState(self, shutter_id, shutter_open):
    """
    Sets the shutter with given id  (0 or 1) to the state given by
    shutter_open (true or false).
    """
    shutter_status = bool(self.gpioState() & (1<<(shutter_id + 8)))
    if shutter_open is shutter_status:
      return
    elif shutter_open:
      self.gpioState(1<<shutter_id)
    else:
      self.gpioState(1<<(shutter_id + 2))
    time.sleep(0.1)
    self.gpioState(0)
