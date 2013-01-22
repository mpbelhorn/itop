"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array
import time

def pauseForStage(stage):
  while stage.getMotionStatus():
    # Hold python execution in null loop until stage is stopped.
    pass
  
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
    
  def search(self, start_point, x_stop, step_size):
    """
    Searches through a range of position steps for the beam.
    
    All arguments given in millimeters.
    The step size is a signed quantity where the sign indicates the direction
    of travel.
    """
    x_start = start_point[0]
    z_coordinate = start_point[1]
    beam_seen = False
    if x_stop > self.upper_limit:
      upper_range = self.upper_limit
    if x_start < self.lower_limit:
      x_start = self.lower_limit
    steps = [[x/1000., z_coordinate] for x in range(
        int(x_start * 1000.),
        int(x_stop * 1000.),
        int(step_size * 1000.))]
    steps.append([x_stop, z_coordinate])
    for position in steps:
      self.controller.groupMoveLine(self.group_id, position)
      while self.controller.groupIsMoving(self.group_id):
        if (self.camera.read()['power'] > self.power_level):
          beam_seen = True
      cam_reading = self.camera.read()
      if (cam_reading['power'] < self.power_level and beam_seen):
        print "Passed the beam."
        return position
      elif (cam_reading['power'] > self.power_level):
        beam_seen = True
        beam_offset = cam_reading['centroid_x'] - (step_size * 500.)
        if (step_size > 0 <= beam_offset) or (step_size < 0 >= beam_offset):
          print "Passed the beam."
          return position
        else:
          continue
    else:
      # Something's not right...
      cam_reading = self.camera.read()
      if cam_reading['power'] > self.power_level:
        if -20 < cam_reading['centroid_x'] < 20:
          print "On the beam within thermal fluctuations."
          return self.controller.groupPosition(self.group_id)
      elif beam_seen:
        print "ERROR: Beam center not in reach of camera center."
      else:
        # The beam was not detected! The beam may be out of range, blocked, or
        # the stage moved too fast to register the beam on camera over with
        # the given serial polling frequency. Also, there may be a bug in the
        # code.
        print "ERROR: Beam not detected."
    self.controller.groupOff(self.group_id)
    return [0, 0] 
    
  def findBeam(self, z_coordinate):
    """
    Centers the beam on a camera attached to given stage group.
    """
    self.controller.groupVelocity(self.group_id, 30)
    start_point = [self.lower_limit, z_coordinate]
    self.controller.groupMoveLine(self.group_id, start_point)
    self.controller.pauseForGroup(self.group_id)
    time.sleep(1)
    self.controller.groupVelocity(self.group_id, 5)
    scan_steps = [50.00, 25.00, 5.00, 1.00, 0.25, 0.12, 0.05, 0.01]
    scan_range = self.upper_limit - self.lower_limit
    for step_number, step_size in enumerate(scan_steps):
      sign = (-1)**step_number
      start_point = self.search(
          start_point, start_point[0] + sign*scan_range, sign*step_size)
      scan_range = 2.0 * step_size
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
    self.beam_crossing_found = False
    
  def moveOnBeam(self, position):
    self.controller.groupMoveLine(self.group_id,
        self.on_beam.position(position))
    
  def findFocalPoint(self, mirror_position):
    self.mirror.on()
    self.mirror.position(250 - mirror_position)
    pauseForStage(self.mirror)
    # Block the free beam. Done manually for now. TODO: get an automatic block.
    self.on_beam.findSlope()
    # Unblock the free beam. Done manually for now.
    self.controller.groupVelocity(self.group_id, 5)