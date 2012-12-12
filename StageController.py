import serial

class StageController(object):
  def __init__(self, serial_device):
    self.io = serial.Serial(serial_device, 19200, timeout = 1)
    self.io_end = '\r'
    self.axis1 = Stage(1, self)
    self.axis2 = Stage(2, self)
    self.axis3 = Stage(3, self)
    
  def send(self, command, parameter = '', axis = ''):
    """Send a command to the controller."""
    self.io.write(axis + command + parameter + self.io_end)
    
  def read(self):
    """Read a line from the controller's serial buffer."""
    print self.io.readline()
    
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

class Stage(object):
  def __init__(self, axis, controller):
    """Initialize the stage. Requires a controller instance."""
    self.axis = str(axis)
    self.controller = controller
    
  def send(self, command, parameter=''):
    """Send a command to this axis."""
    self.controller.send(command, parameter, self.axis)
    
  def on(self):
    """Turns the axis motor on."""
    self.send('MO')
    
  def off(self):
    """Turns the axis motor off."""
    self.send('MF')
    
  def getStageID(self):
    """Get stage model and serial number."""
    #TODO - Check syntax.
    self.send('ID')
    self.controller.read()
    
  def getMotionStatus():
    """Return false for stopped, true for in motion."""
    self.send('MD?')
    self.read()
    
  def goToHome(self):
    """Moves the stage to the home position."""
    self.send('OR')
    
  def homePreset(self, home_position = '?'):
    """Sets the absolute position ascribed to the home position."""
    self.send('SH', home_position)
    if (home_position == '?'):
      self.read()
    
  def position(self, absolute_position = '?'):
    """Moves the stage to an absolute position."""
    self.send('PA', str(absolute_position))
    if (absolute_position == '?'):
      self.read()
    
  def move(self, relative_position):
    """Moves the stage to a reltive position."""
    self.send('PR', str(relative_position))
    
  def setAcceleration(self, acceleration = '?'):
    """Sets te stage acceleration."""
    self.send('AC', str(acceleration))
    if (acceleration == '?'):
      self.controller.read()
    
  def setDeceleration(self, deceleration = '?'):
    """Sets te stage deceleration."""
    self.send('AG', str(deceleration))
    if (deceleration == '?'):
      print self.controller.read()
      
  def setAccelerationLimit(self, limit = '?'):
    """Set maximum acceleration/deceleration limit."""
    self.send('AU', limit)
    if (limit == '?'):
      self.controller.read()
    
  def setVelocity(self, velocity = '?'):
    """Sets the stage velocity."""
    self.send('VA', str(velocity))
    if (velocity == '?'):
      self.controller.read()
      
  def waitUntilPosition(position):
    """Pause processing controller commands until stage is at position."""
    self.send('WS', milliseconds)
      
  def waitUntilStopped(time=''):
    """Pause until elapsed time [milliseconds] after stage is stopped."""
    self.send('WS', time)