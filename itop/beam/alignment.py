# -*- coding: utf-8 -*-
"""
A class to manage the alignment of beams to the tracker.

"""
from itop.beam import Beam
import datetime


class Alignment(object):
  """A class to establish the alignment between a tracker and the beams.

  """
  def __init__(self):
    """Constructor for alignment."""
    self.beam_a = None
    self.beam_b = None
    self.displacement = None  # r_b(x,y,0) - r_a(x,y,0) in tracker frame.
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

