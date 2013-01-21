"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array

class ConstrainToBeam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """
  def __init__(self, controller, group_id, camera, **kwargs):
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    lower_limit = kwargs.pop('lower_limit', -125)
    upper_limit = kwargs.pop('upper_limit',  125)
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
    steps = [[x/1000. + start[0], start[1]]
        for x in range(int(start * 1000.),
        int((stop + signed_step_size) * 1000.),
        int(signed_step_size * 1000.)]
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
    # Perhaps an else statement here to catch missed beam passings would be 
    #   a good idea.
    return position
    
  def findBeam(self, z_coordinate):
    """
    Centers the beam on a camera attached to given stage group.
    """
    self.controller.groupVelocity(self.group_id, 30)
    self.controller.groupMoveLine(self.group_id, [-120, z_coordinate])
    while self.controller.axis2.getMotionStatus():
      pass
    while self.controller.axis3.getMotionStatus():
      pass
    self.controller.groupVelocity(self.group_id, 5)
    position = [-125.0, z_coordinate]
    position = stepStage(position, 125.0, 10.0)
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
    self.r_initial = array(findBeam(self.controller, -120))
    self.r_final = array(findBeam(self.controller, 120))
    self.slope = self.r_final - self.r_initial
    
  def position(self, fraction):
    """
    Moves the stage group along the currently defined trajectory.
    """
    if fraction < 0 or fraction > 1:
      print "Cannot exceed stage limits."
    else:
      return (self.r_initial + fraction * self.slope).tolist()