# -*- coding: utf-8 -*-
"""
A class to manage the alignment of beams to the tracker.

"""
from itop.beam import Beam
from itop.math import Vector
from itop.utilities import clamp
import datetime



class Alignment(object):
  """A class to establish the alignment between a tracker and the beams.

  """

  INTERFERENCE_CUTOFF = 40.0 # mm off the optical axis.

  def __init__(self, calibration):
    """Constructor for alignment."""
    self.beam_a = None
    self.beam_b = None
    self.displacement = None  # r_b(x,y,0) - r_a(x,y,0) in tracker frame.
    self.calibration = calibration
    self.date = None

  def align(self, tracker, home=False):
    """Establishes the alignment between the given tracker and the beams.

    Takes an optional keyword argument
    home (False): Recalibrates the rotation stage to it's home switch.

    """
    print 'Measuring tracker alignment. This will take some time.'
    self.beam_a = Beam(-1)
    self.beam_b = Beam(-1)
    tracker.rotation_stage.power_on()
    if home:
      tracker.rotation_stage.go_to_home(wait=True)
    tracker.rotation_stage.position(180, wait=True)
    tracker.facing_z_direction = 1
    shutter = tracker.driver.shutter_state
    shutter(0, 1)
    shutter(1, 0)
    self.beam_a = tracker.find_beam_trajectory(
        [tracker.axes[0].limits.upper,
         tracker.axes[1].limits.lower + 11,
         tracker.axes[2].limits.upper],
        -1, -1, z_samples=25)
    shutter(0, 0)
    shutter(1, 1)
    self.beam_b = tracker.find_beam_trajectory(
        self.beam_a.last_sample() + [-30, -5, 0],
        -1, 1, z_samples=25)
    self.displacement = self.beam_b.intercept - self.beam_a.intercept
    self.date = datetime.datetime.now().isoformat()

    # Rotate camera to face mirror.
    tracker.rotation_stage.position(0, wait=True)
    tracker.facing_z_direction = -1
    shutter(0, 1)

  def alignment_date(self):
    """Returns the date and time the current alignment data was taken.

    """
    if self.date is None:
      return 'invalid'
    else:
      return self.date

  def base_input_positions(self):
    """Return a list of the input positions for each beam when the mirror is at
    position (0,0,0).
    """
    inputs = [self.calibration.primary_input()]
    for beam, separation in zip((self.beam_b,), (self.displacement,)):
      nominal_input = inputs[0] + separation
      inputs.append(
          nominal_input - (
            nominal_input.dot(-beam.direction) * (-beam.direction)))
    return inputs

  def mirror_positions(self, input_position):
    """Return a list of the mirror positions for each beam to be at the given
    input position.
    """
    return [i - Vector(input_position) for i in self.base_input_positions()]

  def mirror_limits(self, input_range):
    """
    Return the mirror stage position limits that fully accomodate the
    given input x-position range.
    """
    limit_candidates = []
    for scan_limit in input_range:
      beam_pair = []
      for beam in self.mirror_positions([scan_limit, 0, 0]):
        beam_pair.append(clamp(beam[0].value, -250, 250))
      limit_candidates.append(beam_pair)
    if input_range[1] > input_range[0]:
      return (max(limit_candidates[0]), min(limit_candidates[1]))
    else:
      return (min(limit_candidates[0]), max(limit_candidates[1]))

  def input_positions(self, mirror_position):
    """Return a list of the beam input positions given the mirror position."""
    return [i - Vector(mirror_position) for i in self.base_input_positions()]

  def out_of_range(self, beam_index, mirror_position):
    """Return True if the beam with given index cannot be sampled due to any
    of the following conditions:
        profiler interference
        beam misses mirror
    """
    input_x = self.input_positions(mirror_position)[beam_index][0].value
    if (abs(input_x) < Alignment.INTERFERENCE_CUTOFF):
      return True
    elif (input_x < self.calibration.mirror_dimensions[0][0]) or (
          self.calibration.mirror_dimensions[0][1] < input_x):
      return True
    else:
      return False


class Calibration(object):
  """Data required to calibrate the instrumentation."""
  def __init__(self,
      tcal_from_mcal=(0, 0, 1447),
      optical_axis_from_mcal=(-33.22, 0.0, -10.832),
      ccd_from_tcal=(-36.03, 0.0, -3.65),
      tcal_coordinates=(125.0, 0.0, -95.0),
      mcal_coordinates=(-220, 0, 0),
      mirror_dimensions=((-225.0, 225.0), (-20, 0), (0,-100))):
    """
    Constructs a calibration object.

    """
    # mirror_faces - mirror_mount = (-11.61, 0.0, -8.882)
    # mirror_mount - mcal = (-21.61, 0.0, -1.95)
    # ccd - profiler_chassis = (-32.004, 0.0, 6.60)
    #
    self.tcal_from_mcal = Vector(tcal_from_mcal)
    self.optical_axis_from_mcal = Vector(optical_axis_from_mcal)
    self.tcal_coordinates = Vector(tcal_coordinates)
    self.mcal_coordinates = Vector(mcal_coordinates)
    self.mirror_dimensions = mirror_dimensions

  def primary_input(self):
    """Return the input position of the primary beam when the mirror stage
    is at position (0, 0).
    """
    return Vector([0.0, 0.0, 0.0])

  def offset(self):
    # TODO - Nonzero y is sign-reversed in the calculation.
    return -self.tcal_coordinates + self.tcal_from_mcal - self.optical_axis_from_mcal

