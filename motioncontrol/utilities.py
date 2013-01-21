"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array
import time

def pauseForStage(stage):
  while stage.getMotionStatus():
    # Hold python execution in null loop until stage is stopped.
    pass
  return

class ConstrainToBeam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """
  def __init__(self, controller, group_id, camera, **kwargs):
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    # Assume group of ILS250CC stages if no kwargs given.
    self.lower_limit = kwargs.pop('lower_limit', -125)
    self.upper_limit = kwargs.pop('upper_limit',  125)
    self.power_level = kwargs.pop('power_level', 0.002)
    self.r_initial = array([self.lower_limit, self.lower_limit])
    self.r_final = array([self.upper_limit, self.upper_limit])
    travel_range = self.upper_limit - self.lower_limit
    self.slope = array([travel_range, travel_range])
    
  def search(self, start, stop, signed_step_size):
    """
    Searches through a range of position steps for the beam.
    
    All arguments given in millimeters.
    """
    upper_range = stop + signed_step_size
    if upper_range > self.upper_limit or upper_range < self.lower_limit:
      if signed_step_size > 0:
        upper_range = self.upper_limit
      else:
        upper_range = self.lower_limit
    steps = [[x/1000., start[1]] for x in range(
        int(start[0] * 1000.),
        int(stop * 1000.),
        int(signed_step_size * 1000.))]
    print steps
    final_position = start
    beam_seen = False
    for position in steps:
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.axis2.getMotionStatus():
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
          final_position = position
      cam_reading = self.camera.read()
      if (cam_reading['power'] < self.power_level and beam_seen):
        print "passed beam"
        return final_position
      elif (cam_reading['power'] > self.power_level):
        if signed_step_size > 0:
          if cam_reading['centroid_x'] > (signed_step_size * 500.):
            print "passed beam but it is still in view."
            return final_position
        else:
          if cam_reading['centroid_x'] < (signed_step_size * 500.):
            print "passed beam but it is still in view."
            return final_position
    else:
      # The beam was not detected! Scan the last segment not covered in steps.
      # Alternatively, there may be a bug or the stage moved across the beam
      # too fast to register on the camera over serial communication.
      print "Scanning last segment."
      position = [steps[-1], upper_range]
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.axis2.getMotionStatus():
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
          final_position = position
          print "beam seen in last segment."
      cam_reading = self.camera.read()
      if cam_reading['power'] < self.power_level and beam_seen:
        print "passed beam in last segment"
        return final_position
      elif (cam_reading['power'] > self.power_level):
        if signed_step_size > 0:
          if cam_reading['centroid_x'] > (signed_step_size * 500.):
            print "passed beam but it is still in view."
            return final_position
        else:
          if cam_reading['centroid_x'] < (signed_step_size * 500.):
            print "passed beam but it is still in view."
            return final_position
      else:
        print "Beam center not in reach of camera center!"
        self.controller.groupOff(self.group_id)
    return final_position
    
  def findBeam(self, z_coordinate):
    """
    Centers the beam on a camera attached to given stage group.
    """
    self.controller.groupVelocity(self.group_id, 30)
    self.controller.groupMoveLine(
        self.group_id, [self.lower_limit + 5, z_coordinate])
    pauseForStage(self.controller.axis2)
    pauseForStage(self.controller.axis3)
    self.controller.groupVelocity(self.group_id, 5)
    position = [self.lower_limit, z_coordinate]
    position = self.search(position, self.upper_limit, 50.0)
    print "Next scan"
    position = self.search(position, position[0] - 75.0,  -25.0)
    print "Next scan"
    position = self.search(position, position[0] + 50.0,   5.00)
    print "Next scan"
    position = self.search(position, position[0] - 10.00, -1.00)
    print "Next scan"
    position = self.search(position, position[0] + 2.00,   0.25)
    print "Next scan"
    position = self.search(position, position[0] - 1.00,  -0.12)
    print "Next scan"
    position = self.search(position, position[0] + 1.00,   0.05)
    print "Next scan"
    position = self.search(position, position[0] - 1.00,  -0.01)
    print "last scan"
    return self.controller.groupPosition(self.group_id)
  
  def findSlope(self):
    """
    Finds the trajectory of the stages needed to keep a beam centered on camera.
    """
    self.r_initial = array(self.findBeam(self.lower_limit))
    self.r_final = array(self.findBeam(self.upper_limit))
    self.slope = self.r_final - self.r_initial
    
  def position(self, fraction):
    """
    Moves the stage group along the currently defined trajectory.
    """
    if fraction < 0 or fraction > 1:
      print "Cannot exceed stage limits."
    else:
      return (self.r_initial + fraction * self.slope).tolist()
      
class FocalPoint(object):
  def __init__(self, controller, group_id, camera):
    self.controller = controller
    self.mirror = self.controller.axis1
    self.group_id = group_id
    self.camera = camera
    self.on_beam = ConstrainToBeam(self.controller, self.group_id, self.camera)
    
  def findFocalPoint(self, mirror_position):
    self.mirror.on()
    self.mirror.position(250 - mirror_position)
    pauseForStage(self.mirror)
    print "Block 'free' beam now!"
    for i in xrange(15):
      print 15 - i, '\a'
      time.sleep(1)
    print "Scanning for beam."
    self.on_beam.findSlope()
    print "Clear 'free' beam now!"
    for i in xrange(5):
      print 5 - i, '\a'
      time.sleep(1)
    print "Scan for focal point now.", '\a'
    self.controller.groupVelocity(self.group_id, 30)
    
  def moveOnBeam(self, position):
    self.controller.groupMoveLine(self.group_id, self.on_beam.position(position))
    pauseForStage(self.controller.axis2)
    pauseForStage(self.controller.axis3)