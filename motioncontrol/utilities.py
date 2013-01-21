"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array

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
    self.r_initial = array([lower_limit, lower_limit])
    self.r_final = array([upper_limit, upper_limit])
    travel_range = upper_limit - lower_limit
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
    steps = [[x/1000. + start[0], start[1]]
        for x in range(
        int(start[0] * 1000.), int(stop * 1000.), int(signed_step_size * 1000.)]
    final_position = start
    beam_seen = False
    for position in steps:
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.axis2.getMotionStatus():
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
          final_position = position
        else if beem_seen:
          break
      cam_reading = self.camera.read()
      if (cam_reading['power'] > self.power_level and
          cam_reading['centroid_x'] - (signed_step_size * 1000) > 5):
        final_position = position
        break
    else:
      # The beam was not detected! Scan the last segment not covered in steps.
      # Alternatively, there may be a bug or the stage moved across the beam
      # too fast to register on the camera over serial communication.
      position = [steps[-1], upper_range]
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.axis2.getMotionStatus():
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
          final_position = position
      cam_reading = self.camera.read()
      if cam_reading['power'] > self.power_level:
        if cam_reading['centroid_x'] - (signed_step_size * 1000) > 5):
          final_position = position
        else:
          print "Beam center not in reach of camera center!"
          self.controller.groupOff(self.group_id)
    return final_position
    
  def findBeam(self, z_coordinate):
    """
    Centers the beam on a camera attached to given stage group.
    """
    self.controller.groupVelocity(self.group_id, 30)
    self.controller.groupMoveLine(self.group_id, [-120, z_coordinate])
    pauseForStage(self.controller.axis2)
    pauseForStage(self.controller.axis3)
    self.controller.groupVelocity(self.group_id, 5)
    position = [self.lower_limit, z_coordinate]
    position = stepStage(position, self.upper_limit, 10.0)
    self.controller.groupVelocity(self.group_id, 30)
    position = stepStage(position, position[0] - 10.0, -5.00)
    position = stepStage(position, position[0] + 5.00,  1.00)
    position = stepStage(position, position[0] - 1.00, -0.25)
    position = stepStage(position, position[0] + 0.25,  0.05)
    position = stepStage(position, position[0] - 0.05, -0.01)
    return self.controller.groupPosition(self.group_id)
  
  def findSlope(self):
    """
    Finds the trajectory of the stages needed to keep a beam centered on camera.
    """
    self.r_initial = array(findBeam(self.controller, self.lower_limit))
    self.r_final = array(findBeam(self.controller, self.upper_limit))
    self.slope = self.r_final - self.r_initial
    
  def position(self, fraction):
    """
    Moves the stage group along the currently defined trajectory.
    """
    if fraction < 0 or fraction > 1:
      print "Cannot exceed stage limits."
    else:
      return (self.r_initial + fraction * self.slope).tolist()