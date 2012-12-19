import serial

class StageController(object):
  """
  Encompasses serial I/O for Newport EPS300 motion controllers, controller
  operation methods plus instances of the stage I/O classes for each
  axis attached to the controller.
  """
  
  def __init__(self, serial_device):
    """
    Creates an I/O handle on a Newport EPS300 motion controller and its axes.
    
    Arguments:
    serial_device -- Path string to serial port used by the controller.
    """
    self.io = serial.Serial(serial_device, 19200, timeout = 1)
    self.io_end = '\r'
    self.axis1 = Stage(1, self)
    self.axis2 = Stage(2, self)
    self.axis3 = Stage(3, self)
    self.readFirmwareVersion()
    
  def send(self, command, parameter = '', axis = ''):
    """Send a command to the controller."""
    self.io.write(axis + command + parameter + self.io_end)
    
  def read(self):
    """Return a line read from the controller's serial buffer."""
    return self.io.readline()
    
  def reset(self):
    """Perform a full controller reset."""
    self.send('RS')
    
  def readStatus(self):
    """Read the first error message in the error FIFO buffer."""
    self.send('TS')
    self.read()
    
  def readActivity(self):
    """Read the activity register."""
    self.send('TX')
    self.read()
    
  def readError(self):
    """Read the first error message in the error FIFO buffer."""
    self.send('TB?')
    self.read()
    
  def readFirmwareVersion(self):
    """Report the controller firmware version."""
    self.send('VE?')
    self.read()
    
  def wait(milliseconds='0'):
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

class Stage(object):
  def __init__(self, axis, controller):
    """Initialize the stage. Requires a controller instance."""
    self.axis = str(axis)
    self.controller = controller
    
  #-----------------------------------------------------------------------------
  # I/O methods
  #-----------------------------------------------------------------------------
  
  def send(self, command, parameter=''):
    """Send a command to this axis."""
    self.controller.send(command, str(parameter), self.axis)
    
  #-----------------------------------------------------------------------------
  # Status Functions
  #-----------------------------------------------------------------------------

  def targetedPosition(self):
    """Returns the position the stage is currently targeting."""
    self.send('DP?')
    position = self.controller.read()
    print position, self.units()
    return float(position)
    
  def targetedVelocity(self):
    """Returns the stage's targeted velocity."""
    self.send('DV')
    velocity = self.controller.read()
    print velocity+self.units()+'/s'
    return float(velocity)
    
  def stageID(self):
    """Returns stage model and serial number."""
    self.send('ID')
    return self.controller.read()
    
  def getMotionStatus():
    """Return false for stopped, true for in motion."""
    self.send('MD?')
    return self.controller.read()
  
  
  #-----------------------------------------------------------------------------
  # Motion and Position Control
  #-----------------------------------------------------------------------------
  def on(self):
    """Turns the axis motor on."""
    self.send('MO')
    
  def off(self):
    """Turns the axis motor off."""
    self.send('MF')
  
  def defineHome(self, position = '?'):
    """Sets the stage home position to given position in current units."""
    self.send('DH', position)
    if (position == '?'):
      position = self.controller.read()
      print position+self.units()
    return float(position)
    
  def moveToLimit(self, direction = '?'):
    """Given the argument '+' or '-', moves stage that hardware limit."""
    self.send('MT', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
  
  def moveIndefinately(self, direction = '?'):
    """Initiates continuous motion in the given '+' or '-' direction."""
    self.send('MV', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
    
  def moveToNextIndex(self, direction = '?'):
    """Moves to the nearest index in the given '+' or '-' direction."""
    self.send('MZ', position)
    if (position == '?'):
      finishedQ = self.controller.read()
      return int(finishedQ)
      
  def goToHome(self):
    """Moves the stage to the home position."""
    self.send('OR')
    
  def position(self, absolute_position = '?'):
    """Moves the stage to an absolute position."""
    self.send('PA', absolute_position)
    if (absolute_position == '?'):
      absolute_position = self.controller.read()
      print absolute_position, self.units()
    return float(absolute_position)
    
  def move(self, relative_position):
    """Moves the stage the given relative position."""
    self.send('PR', relative_position)
    return float(relative_position)
    
  def stop(self):
    """Stops motion on this axis with predefined acceleration."""
    self.send('ST')
  
  #-----------------------------------------------------------------------------
  # Motion Device Parameters
  #-----------------------------------------------------------------------------

  def followingError(self, error = '?'):
    """Sets or returns the maximum following error threshold."""
    self.send('FE', error)
    if (error == '?'):
      error = self.controller.read()
      print error+self.units()
    return float(error)
  
  def stepResoltion(self, resolution = '?'):
    """
    Sets or returns the encoder full-step resolution for a Newport Unidrive
      compatible programmable driver with step motor axis.
    """
    self.send('FR', resolution)
    if (resolution == '?'):
      resolution = self.controller.read()
      print resolution+self.units()
    return float(resolution)
    
  def gearRatio(self, gear_ratio = '?'):
    """
    Sets or returns the master-slave reduction ratio for a slave axis.
    
    Use this command very carefully. The slave axis will have its speed and
    acceleration in the same ratio as the position.
    Also, ensure that the ratio used for the slave axis does not cause
    overflow of this axis’ parameters (speed, acceleration), especially with
    ratios greater than 1. 
    """
    self.send('GR', gear_ratio)
    if (gear_ratio == '?'):
      gear_ratio = self.controller.read()
      print gear_ratio+self.units()
    return float(gear_ratio)
  
  # Torque reduction.
  # Microstep factor.
  # Tachometer constant.
  # Motor voltage.
  # Master-slave jog update interval.
  # Slave axis jog velocity coefficients.
  # Left limit.
  def units(self, units = '?'):
    """
    Sets the stage displacement units from given integer.
    If no argument is given, current unit setting is reported.
    
    Possible units:
    0 -- Encoder counts         6 -- micro-inches
    1 -- Motor steps            7 -- degrees
    2 -- millimeters            8 -- gradient
    3 -- micrometers            9 -- radians
    4 -- inches                10 -- milliradian
    5 -- mils (milli-inches)   11 -- microradian
    """
    self.send('SN', units)
    if (units == '?'):
      response = self.controller.read()
      units = ['encoder-counts', 'motor-steps', 'mm', u'\u03BCm', 'in', 'mil',
               u'\u03BCin', u'\u00B0', 'grade', 'rad', 'mrad', u'\u03BCrad']
      return units[int(response)]
  # Right limit.
  # Master-slave relationship.
  # Encoder resolution.
  
  #-----------------------------------------------------------------------------
  # On-board Programming
  #-----------------------------------------------------------------------------
  # Not implemented.
  
  #-----------------------------------------------------------------------------
  # Trajectory Definition
  #-----------------------------------------------------------------------------
  def acceleration(self, acceleration = '?'):
    """Sets the stage acceleration."""
    self.send('AC', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def eStopAcceleration(self, acceleration = '?'):
    """Sets the stage emergency stop acceleration."""
    self.send('AE', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def deceleration(self, deceleration = '?'):
    """Sets te stage deceleration."""
    self.send('AG', deceleration)
    if (deceleration == '?'):
      deceleration = self.controller.read()
      print deceleration+self.units()+'/s^2'
    return float(deceleration)

  def accelerationLimit(self, acceleration = '?'):
    """
    Sets the maximum allowed stage acceleration/deceleration.
    
    Stage will error out if this limit is exceeded.
    """
    self.send('AU', acceleration)
    if (acceleration == '?'):
      acceleration = self.controller.read()
      print acceleration+self.units()+'/s^2'
    return float(acceleration)
    
  def backlashCompensation(self, compensation = '?'):
    """
    Set or report the backlash compensation in current units.
    
    Maximum compensation is equivelent of 10000 encoder counts.
    """
    self.send('BA', compensation)
    if (compensation == '?'):
      compensation = self.controller.read()
      print compensation+self.units()
    return float(compensation)
    
  # Linear Compensation.
  # Jog high speed.
  # Jerk rate.
  # Jog low speed.
  # home search speed low.
  # home search high speed.
  # Home search mode.
  
  def homePreset(self, home_position = '?'):
    """Sets the absolute position ascribed to the home position."""
    self.send('SH', home_position)
    if (home_position == '?'):
      home_position = self.controller.read()
      print home_position+self.units()
    return float(home_position)
    
  # Update filter parameters.
  
  def velocity(self, velocity = '?'):
    """Sets the stage velocity."""
    self.send('VA', velocity)
    if (velocity == '?'):
      velocity = self.controller.read()
      print velocity+self.units()+'/s'
    return float(velocity)
  
  # Base velocity for stepper motors.
  
  def velocityLimit(self, velocity = '?'):
    """
    Sets the maximum allowed stage velocity.
    
    Stage will error out if this limit is exceeded.
    """
    self.send('VU', velocity)
    if (velocity == '?'):
      velocity = self.controller.read()
      print velocity, self.units()+'/s'
    return float(velocity)
  
    
  #-----------------------------------------------------------------------------
  # Flow Control and Sequencing
  #-----------------------------------------------------------------------------
  # Define label.
  # Jump to label.
  # Generate service request.
  # set device address.
  def waitUntilPosition(position):
    """
    Pause EPS command execution until stage is at position.
    
    This does not pause execution of python code!
    """
    self.send('WP', position)
      
  def waitUntilStopped(time=''):
    """
    Pause EPS command execution time [ms] after stage is stopped.
    
    This does not pause execution of python code!
    """
    self.send('WS', time)
  
  #-----------------------------------------------------------------------------
  # Group Functions.
  #-----------------------------------------------------------------------------
  # Set group acceleration.
  # Read list of groups.
  # Move along arc.
  # Set group decel.
  # Set group e-stop decel.
  # Group power off.
  # Set group jerk.
  # Move along line.
  # Create new group.
  # Group power on.
  # Get group position.
  # Wait for group via point buffer.
  # Stop group.
  # Set group velocity.
  # Wait for group to stop.
  # Delete group.
  # Get group size.
  
  #-----------------------------------------------------------------------------
  # Digital Filters
  #-----------------------------------------------------------------------------
  # Not implememted.
  
  #-----------------------------------------------------------------------------
  # Master-Slave Mode Definition
  #-----------------------------------------------------------------------------
  # Set master-slave ratio.
  # Set master-slave jog update interval.
  # master-slave jog velocity coefficients.
  # master-slave mode.
  #

      