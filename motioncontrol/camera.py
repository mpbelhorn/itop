"""
A set of scripts to read the HD-LBP serial stream
"""
import serial
import threading

def readCamera(device):
  cam = serial.Serial(device, 9600, timeout=1)
  keys = ['time', 'centroid_x', 'centroid_y', 'centroid_r', 
          'level_1', 'level_2', 'level_3',
          'width_1', 'width_2', 'width_3',
          'height_1', 'height_2', 'height_3',
          'power']
  buffer = ''
  while True:
    buffer = buffer + cam.read(cam.inWaiting())
    if buffer.count('\n') > 2:
      lines = buffer.split('\n') # Guaranteed to have at least 2 entries
      last_line = lines[-2]
      # buffer = lines[-1]
      header, values = last_line.split("R ",1)
      floats = [float(x) for x in values.split()]
      output = dict(zip(keys, floats))
      return output

def findBeam(controller, z):
  for position in range(-120,120):
    controller.groupMoveLine(1, [position, z])
    while controller.axis2.getMotionStatus():
      pass
    while controller.axis3.getMotionStatus():
      pass
    if readCamera('/dev/ttyUSB1')['power'] > 0.005:
      fine_range = [position + x/20. for x in range(0, 6*20)]
      break
  for position in fine_range:
    controller.groupMoveLine(1, [position, z])
    while controller.axis2.getMotionStatus():
      pass
    if abs(readCamera('/dev/ttyUSB1')['centroid_x']) < 100:
      return controller.groupPosition(1)