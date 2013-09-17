"""
A module for reading out and manpilating photodiodes via a Hamamatsu C9329
amplifier.
"""
import serial
from time import sleep

class PhotodiodeError(Exception):
  """Error and exception handling for the Photodiode class."""
  pass

class Photodiode(object):
  """A photodiode attached to a Hamamatsu C9329 amplifer."""
  def __init__(self, device):
    """Construct a photodiode object from the given serial device."""
    self.serial = serial.Serial(
        device, baudrate=19200, stopbits=1, bytesize=8,
        parity=serial.PARITY_NONE)

  def read(self):
    """Return the current output of the photodiode."""
    self.serial.flushInput()
    readout = ''
    while True:
      readout = readout + self.serial.read(self.serial.inWaiting())
      lines = readout.split()
      if len(lines) > 2:
        if lines[1][0] is not '*':
          return int(lines[1].split(',')[0], 16) * 5.0 / 0x7fff
        else:
          readout = ''

  def _issue_command(self, command):
    """Send a raw command string to the amplifier and return the response."""
    self.serial.flushInput()
    self.serial.flushOutput()
    self.serial.write(command)
    sleep(0.1)
    return self.serial.read(
        self.serial.inWaiting()).partition('*')[-1].split()[0]

  def read_continuous(self):
    """Continuously print the current sensor reading."""
    print("Press Ctrl-C to exit")
    try:
      while True:
        print(self.read())
    except KeyboardInterrupt:
      pass

  def continuous_mode(self):
    """Put the device in continuous readout mode."""
    return self._issue_command('*MOD0\n')

  def logger_mode(self, interval=50, scale=1, count=200):
    """Put the device in logger mode."""
    log_response = self._issue_command('*MOD1\n')
    interval_response = self._measurement_interval(interval, scale)
    count_response = self._measurement_count(count)
    return (log_response, interval_response, count_response)

  def _measurement_interval(self, interval, scale):
    """Set the logger mode measurement interval."""
    return self._issue_command('*BTW{:04}{:1}\n'.format(interval, scale))

  def _measurement_count(self, count):
    """Set the logger mode measurement count."""
    return self._issue_command('*CNT{}\n'.format(count))

  def clear_logger_data(self):
    """Clear data stored in the logger data buffer."""
    return self._issue_command('*CLR\n')

  def number_of_logged_data(self):
    """Return the number of measurements stored in the logger data buffer."""
    return self._issue_command('*ASK\n')

  def get_logged_data(self):
    """Return the data stored in the logger data buffer."""
    self._issue_command('*REQ\n')

  def zero(self):
    """Set the zero-offset to tare the device to the current reading."""
    self._issue_command('*ZER\n')

  def version(self):
    """Return the amplifier version number."""
    return self._issue_command('*VER\n')

  def settings(self):
    """Return the amplifer settings."""
    return self._issue_command('*SET\n')



