"""
A class to read the data from a Newport HD-LBP laser beam profiler.
"""
import serial

class LaserBeamProfiler(object):
  """
  Provides an interface to a Newport HD-LBP over serial link.
  """
  def __init__(self, device):
    """
    Establish serial communication with an HD-LBP.
    """
    self.device = device
    self.io = serial.Serial(device, 9600, timeout=1)
    self.keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r', 
                 'level_1', 'level_2', 'level_3',
                 'width_1', 'width_2', 'width_3',
                 'height_1', 'height_2', 'height_3',
                 'power']
  
  def read(self):
    """
    Read last complete line sent by HD-LBP.
    
    The output is in the form of a dictionary. Dictionary keys are given in
    the class ctor.
    """
    buffer = ''
    while True:
      buffer = buffer + self.io.read(self.io.inWaiting())
      if '\n' in buffer:
        lines = buffer.split('\n')
        if lines[-2]:
          last_full_line = lines[-2]
          header, values = last_full_line.split("R ",1)
          floats = [float(x) for x in values.split()]
          output = dict(zip(self.keys, floats))
          return output
        buffer = lines[-1]        