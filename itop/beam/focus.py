# -*- coding: utf-8 -*-
"""
A module for tracking the focal point of a spherical mirror.

"""

import time
from itop.beam.beam import Beam


class DataPoint(object):
  """Finds, calculates and returns information about the focal point
  and beam crossing positions in general of two beams.

  """
  def __init__(self, tracker, mirror):
    """Constructor for FocalPoint.

    Requires a beam tracker and mirror motion axis.

    """
    self.tracker = tracker
    self.mirror = mirror
    self.beam_a = Beam()
    self.beam_b = Beam()

  def find_trajectories(self, proximal=False):
    """Initilizes the trajectories of both beams.

    Takes one keyword argument.
    proximal -- Boolean (False). If true, the last trajectory data is used
                to narrow the search for the new trajectory.

    """
    start_point = [
        self.tracker.axes[0].limits.lower,
        20.,
        self.tracker.axes[2].limits.lower]
    if proximal and self.beam_a.direction is not None:
      start_point = self.beam_a.first_sample() + [-25, 0, 0]
    # Block beam 'B' and find beam 'A' trajectory.
    shutter = self.tracker.driver.shutter_state
    shutter(0, 0)
    shutter(1, 1)
    time.sleep(0.25)
    self.beam_a = self.tracker.find_beam_trajectory(start_point)
    # Block beam 'A' and find beam 'B' trajectory.
    shutter(1, 0)
    shutter(0, 1)
    time.sleep(0.25)
    self.beam_b = self.tracker.find_beam_trajectory(
        self.beam_a.last_sample() + [-20, 0, 0], scan_direction_z=-1)
    shutter(1, 1)


  def find_focal_points(self, mirror_position, refresh=False, proximal=False):
    """Finds the focal points (assuming astigmatism) of the beams.

    Takes two optional keyword arguments.
    refresh  -- Boolean (False). If true, the trajectory data is cleared
                and the beams are relocated.
    proximal -- Boolean (False). Implies refresh. The beams are relocated
                assuming they are very near the last trajectories.

    """
    self.mirror.position(mirror_position, wait=True)

    # Initialize the beam trajectories if necessary.
    if proximal:
      self.find_trajectories(proximal=True)
    elif ((self.beam_a.direction is None) or
        (self.beam_b.direction is None) or
        refresh):
      self.find_trajectories()
    return (self.beam_a, self.beam_b, self.mirror)

