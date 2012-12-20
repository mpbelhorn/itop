"""
This module provides methods for controlling and communicating with an EPS300
motion controller.
"""

import serial
import Stage

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