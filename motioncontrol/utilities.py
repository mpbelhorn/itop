"""
Utility classes for iTOP mirror measurements.
"""
from numpy import array, std, mean, dot
from numpy import linalg
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
    self.power_level = kwargs.pop('power_level', 0.004)
    self.r_initial = None
    self.r_final = None
    self.slope = None
    self.slope3D = None # (x,y,z)

  def position(self, fraction=None):
    """
    Returns the 3D position of the beam at the current 2D stage position.

    If given an optional argument 'fraction', returns the coordinates of the
    beam at the given fraction of the available stage group trajectory.
    """
    position = None
    if fraction is not None:
      try:
        position = (self.r_initial + fraction * self.slope3D).tolist()
      except TypeError:
        print "Trajectory is not initilized"
    else:
      profile = self.camera.read()
      if (profile['power'] > self.power_level):
        position = self.controller.groupPosition(self.group_id)
        elevation = profile['centroid_y'] / 1000.0
        position.insert(1, elevation)
      else:
        print "Beam not visible"
    return position

  def centerBeam(self):
    """
    Centers the camera on the beam if beam is visible.
    """
    if self.camera.read()['power'] >= self.power_level:
      jitter = [self.camera.read()['centroid_x'] for i in range(10)]
      beam_x = mean(jitter)
      # TODO - Change addition or subtraction based on camera direction.
      self.controller.groupMoveLine(self.group_id, (
          array(self.controller.groupPosition(self.group_id)) +
          array([beam_x / 1000.0, 0])), wait=True)
      return True
    else:
      print "ERROR: Insufficient exposed power."
      return False

  def findBeam(self, position):
    """
    Scans in X for single beam and centers camera on beam.
    """
    # Move camera into starting point.
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, [-125, position], wait=True)
    time.sleep(0.125)
    self.controller.groupVelocity(self.group_id, 10)

    # Scan for beam crossing.
    self.controller.groupMoveLine(self.group_id, [125, position])
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
    print beam_positions
    beam_x = mean(beam_positions, 0)[0]
    self.controller.groupMoveLine(self.group_id, [beam_x, position], wait=True)
    self.controller.groupVelocity(self.group_id, 10)
    if not self.centerBeam():
      self.controller.groupMoveLine(
          self.group_id, [beam_x - 3.0, position], wait=True)
      if not self.centerBeam():
        self.controller.groupMoveLine(
            self.group_id, [beam_x + 6.0, position], wait=True)
    if self.centerBeam():
      print "Beam found"
    return self.position()

  def findTrajectory(self):
    """
    Find trajectory of single beam.
    """
    # Find the beam at most upstream position.
    self.r_initial = None
    while self.r_initial is None:
      self.r_initial = array(self.findBeam(-125))

    # Calculate rough trajectory of the beam.
    step = self.r_initial + array([0, 0, 30])
    self.controller.groupMoveLine(self.group_id, step[0::2], wait=True)
    self.centerBeam()
    upstream_sample = self.position()
    downstream_sample = [self.r_initial[0] + (
        ((self.upper_limit_z - self.r_initial[2]) /
        (upstream_sample[2] - self.r_initial[2])) *
        (upstream_sample[0] - self.r_initial[0])),
        self.r_initial[1],
        self.upper_limit_z]
    self.controller.groupVelocity(self.group_id, 40)
    self.controller.groupMoveLine(self.group_id, downstream_sample[0::2], wait=True)
    self.controller.groupVelocity(self.group_id, 10)

    # Refine trajectory of the beam.
    self.centerBeam()
    self.r_final = array(self.position())
    self.slope3D = self.r_final - self.r_initial
    self.slope = self.slope3D[0::2]
    return self.slope3D

  def moveOnBeam(self, fraction):
      """
      Moves the stage group along the beam trajectory to the given fraction
      of available group path.
      """
      self.controller.groupMoveLine(self.group_id,
          (self.position(fraction))[0::2])

  def angle(self):
    """
    Returns the angle in radians of the outgoing beam relative to the
    stage z-axis.
    """
    return (self.slope3D / self.slope[1])

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
    raw_input("Clear(Block) beam 'A'('B'). Press enter to continue.")
    self.beam_a.findTrajectory()
    # Block beam 'A' and find beam 'B' trajectory.
    raw_input("Clear(Block) beam 'B'('A'). Press enter to continue.")
    self.beam_b.findTrajectory()

  def findFocalPoints(self, refresh=False):
    """
    Finds the focal points (assuming astigmatism) of the beams.
    """
    # Initialize the beam trajectories if necessary.
    if (self.beam_a.slope3D is None) or (self.beam_b.slope3D is None) or refresh:
      self.findTrajectories()
    else:
      print "Trajectories already initialized."
    # Find the tangential focal plane.
    # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
    slope_a = self.beam_a.slope3D
    upstream_a = self.beam_a.r_initial
    slope_b = self.beam_b.slope3D
    upstream_b = self.beam_b.r_initial
    tangential_coefficients = array([
        [slope_a[2], -slope_a[0]], [slope_b[2], -slope_b[0]]])
    tangential_ordinates = array([
        slope_a[2] * upstream_a[0] - slope_a[0] * upstream_a[2],
        slope_b[2] * upstream_b[0] - slope_b[0] * upstream_b[2]])
    tangential_solution = linalg.solve(
        tangential_coefficients, tangential_ordinates)
    # Find the sagittal focal plane.
    sagittal_coefficients = array([
        [slope_a[2], -slope_a[1]], [slope_b[2], -slope_b[1]]])
    sagittal_ordinates = array([
        slope_a[2] * upstream_a[1] - slope_a[1] * upstream_a[2],
        slope_b[2] * upstream_b[1] - slope_b[1] * upstream_b[2]])
    sagittal_solution = linalg.solve(
        sagittal_coefficients, sagittal_ordinates)
    # Find the circle of least confusion.
    #   This will have to wait until the camera can be remotely
    #   positioned vertically. Until then, the vertical spot width data
    #   needed to locate the CoLC is unreliable.
    beam_a_tangential_fraction = (tangential_solution[1] - upstream_a[2]) / slope_a[2]
    beam_b_tangential_fraction = (tangential_solution[1] - upstream_b[2]) / slope_b[2]
    self.tangential_focus_a = self.beam_a.position(beam_a_tangential_fraction)
    self.tangential_focus_b = self.beam_b.position(beam_b_tangential_fraction)
    beam_a_sagittal_fraction = (sagittal_solution[1] - upstream_a[2]) / slope_a[2]
    beam_b_sagittal_fraction = (sagittal_solution[1] - upstream_b[2]) / slope_b[2]
    self.sagittal_focus_a = self.beam_a.position(beam_a_sagittal_fraction)
    self.sagittal_focus_b = self.beam_b.position(beam_b_sagittal_fraction)

def refract(ray, normal, origin_index, final_index):
  """
  Returns the normalized direction of a given ray (normalized or not) after
  refraction through a boundary between two media with given normal vector and
  indexes of refraction
  """
  d = array(ray) / linalg.norm(ray)
  n = array(normal) / linalg.norm(normal)
  index_ratio = origin_index / final_index
  incidence = dot(-d, n)
  complement = math.sqrt(1.0 - index_ratio**2 * (1.0 - incidence**2))
  sign = 1.0
  if incidence < 0:
    sign = -1.0
  return index_ratio * d + sign * (index_ratio * incidence - complement) * n

def reconstructMirrorNormal(upstream_ray, **kwargs):
  """
  Reconstructs the mirror normal vector given the downstream ray
  (incoming to front face) beam propagation vector and the upstream
  ray (outgoing from front face).
  """
  downstream_ray = kwargs.pop('downstream_ray', [0, 0, -1])
  face_normal = kwargs.pop('face_normal', [0, 0, 1])
  index_outside = kwargs.pop('index_outside', 1.000277)
  index_inside = kwargs.pop('index_inside', 1.4608)
  refraction = refract(
      downstream_ray, face_normal, index_outside, index_inside)
  reflection = -refract(
      -upstream_ray, face_normal, index_outside, index_inside)
  return (reflection - refraction) / (linalg.norm(reflection - refraction))

class LbpBeamAlignment(object):
  """
  For establishing alignment and positioning of LBP with respect to beams.
  """
  def __init__(self, controller, group_id, camera):
    """
    Set pointers to the LBP and stage group controller.
    """
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
