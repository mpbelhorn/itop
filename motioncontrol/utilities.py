"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array, std, mean
import time
import math

def pauseForStage(stage):
  """
  Hold python execution in null loop until stage is stopped.
  """
  while stage.getMotionStatus():
    pass

def clamp(value, min_value, max_value):
  """
  Constrain a value to between a minimum and maximum.
  """
  return max(min(max_value, value), min_value)

class ConstrainToBeam(object):
  """
  For constaining the movement of a robotic stage group + camera to keep a
  laser beam centered on the camera.
  """

  def __init__(self, controller, group_id, camera, **kwargs):
    """
    Initialize a constraint on the movement of a LBP on a stage group to travel
    along a beam.

    Keyword arguments accepted to constrain the region of group travel.
    Assume group of ILS250CC stages if no kwargs given. Units are those
    that the stages are currently programmed to.

    Option=default values are as follows:
    lower_limit_x=-125 - Lower travel limit.
    lower_limit_z=-125 - Lower travel limit.
    upper_limit_x=125 - Upper travel limit.
    upper_limit_z=125 - Upper travel limit.
    power=level - Beam-in-view power threshold.
    """
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    self.lower_limit_x = kwargs.pop('lower_limit_x', -125)
    self.upper_limit_x = kwargs.pop('upper_limit_x',  125)
    self.lower_limit_z = kwargs.pop('lower_limit_z', -125)
    self.upper_limit_z = kwargs.pop('upper_limit_z',  125)
    self.power_level = kwargs.pop('power_level', 0.003)
    self.r_initial = None
    self.r_final = None
    self.slope = None
    self.slope3D = None # (x,y,z)

  def centerBeam(self):
    """
    Centers the camera on the beam if beam is visible.
    """
    if self.camera.read()['power'] > self.power_level:
      jitter = [self.camera.read()['centroid_x'] for i in range(10)]
      beam_x = mean(jitter)
      self.controller.groupMoveLine(self.group_id, (
          array(self.controller.groupPosition(self.group_id)) -
          array([beam_x / 1000.0, 0])), wait=True)

  def search(self, start_point, stop_point, step_size):
    """
    Searches through a range of position steps for the beam.

    All arguments given in millimeters.
    """
    beam_seen = False
    x_start = clamp(start_point[0], self.lower_limit_x, self.upper_limit_x)
    x_stop = clamp(stop_point[0], self.lower_limit_x, self.upper_limit_x)
    z_start = clamp(start_point[1], self.lower_limit_z, self.upper_limit_z)
    z_stop = clamp(stop_point[1], self.lower_limit_z, self.upper_limit_z)
    displacement = math.hypot((x_stop - x_start), (z_stop - z_start))
    if displacement == 0:
      return None
    count = int(math.floor(displacement/abs(float(step_size))))
    x_step = (x_stop - x_start) / float(count)
    z_step = (z_stop - z_start) / float(count)
    x_steps = [x_start + x_step*i for i in xrange(count)]
    z_steps = [z_start + z_step*i for i in xrange(count)]
    steps = map(list, zip(x_steps, z_steps))
    steps.append([x_stop, z_stop])
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
        beam_offset = cam_reading['centroid_x'] - (x_step * 500.)
        if (x_step > 0 <= beam_offset) or (x_step < 0 >= beam_offset):
          print "Passed the beam."
          return position
    else:
      print "Something's not right..."
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

  def findBeam(self, position):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, [position, -125], wait=True)
    time.sleep(0.125)
    self.controller.groupVelocity(self.group_id, 10)

    # Scan for beam crossing.
    self.controller.groupMoveLine(self.group_id, [125, -125])
    beam_positions = []
    while self.controller.groupIsMoving(self.group_id):
      if (self.camera.read()['power'] > self.power_level):
        beam_positions.append(self.controller.groupPosition(self.group_id))
      elif beam_positions:
        # TODO - Fix unresolved following error on command to stop.
        # self.controller.groupStop(self.group_id)
        self.controller.groupVelocity(self.group_id, 40)
        break
    self.controller.pauseForGroup(self.group_id)
    time.sleep(0.125)
    if not beam_positions:
      print "Beam not seen!"
      return None
    # Go back to the beam.
    beam_x = mean(beam_positions, 0)[0]
    self.controller.groupMoveLine(self.group_id, [beam_x, -125], wait=True)
    self.controller.groupVelocity(self.group_id, 10)
    self.centerBeam()
    self.centerBeam()
    return self.controller.groupPosition(self.group_id)

  def findTrajectory(self):
    """
    Find trajectory of single beam.
    """
    # Find the beam at most downstream position.
    self.r_initial = self.findBeam(-125)
    downstream_elevation = self.camera.read()['centroid_y'] / 1000.0

    # Calculate rough trajectory of the beam.
    self.controller.groupMoveLine(self.group_id,
        self.r_initial + array([0, 30]), wait=True)
    self.centerBeam()
    downstream_sample = self.controller.groupPosition(self.group_id)
    print downstream_sample
    upstream_sample = [self.r_initial[0] + (
        ((self.upper_limit_z - self.r_initial[1]) /
        (downstream_sample[1] - self.r_initial[1])) *
        (downstream_sample[0] - self.r_initial[0])), self.upper_limit_z]
    print upstream_sample
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, upstream_sample, wait=True)
    self.controller.groupVelocity(self.group_id, 10)

    # Refine trajectory of the beam.
    self.centerBeam()
    self.r_final = array(self.controller.groupPosition(self.group_id))
    self.slope = self.r_final - self.r_initial
    upstream_elevation = self.camera.read()['centroid_y'] / 1000.0
    self.slope3D = array([
        self.slope[0],
        upstream_elevation - downstream_elevation,
        self.slope[1]])
    return self.slope3D

  def position(self, fraction):
    """
    Returns the XZ coordinates of the stage group at given fraction of the
    trajectory.
    """
    try:
      return (self.r_initial + clamp(fraction, 0, 1) * self.slope).tolist()
    except TypeError:
      print "Trajectory is not initilized"
      return None

  def moveOnBeam(self, fraction):
    self.controller.groupMoveLine(self.group_id,
        self.position(fraction))

  def angle(self):
    """
    Returns the angle in radians of the outgoing beam relative to the
    stage z-axis.
    """
    return math.atan(self.slope[0] / self.slope[1])

class FocalPoint(object):
  """
  Finds, calculates and returns information about the focal point
  and beam crossing positions in general of two beams.
  """
  def __init__(self, controller, group_id, camera, **kwargs):
    self.controller = controller
    self.mirror = self.controller.axis1
    self.group_id = group_id
    self.camera = camera

    self.beam_a = ConstrainToBeam(self.controller, self.group_id, self.camera)
    self.beam_b = ConstrainToBeam(self.controller, self.group_id, self.camera)

    self.lower_limit_x = kwargs.pop('lower_limit_x', -125)
    self.upper_limit_x = kwargs.pop('upper_limit_x',  125)
    self.lower_limit_z = kwargs.pop('lower_limit_z', -125)
    self.upper_limit_z = kwargs.pop('upper_limit_z',  125)
    self.power_level = kwargs.pop('power_level', 0.003)

  def findTrajectories(self):
    """
    Initilizes the trajectories of both beams.
    """
    # Block beam 'B' and find beam 'A' trajectory.
    raw_input("Block beam 'B'. Press any key to continue.")
    self.beam_a.findTrajectory()
    # Block beam 'A' and find beam 'B' trajectory.
    raw_input("Block beam 'A'. Press any key to continue.")
    self.beam_b.findTrajectory()

  def findFocalPoints(self):
    """
    Finds the focal points (assuming astigmatism) of the beams.
    """
    # Initialize the beam trajectories if necessary.
    if self.beam_a.slope3D is None:
      pass
    # Find the sagittal focal plane.
    # Find the meridional focal plane.
    # Find the circle of least confusion.
