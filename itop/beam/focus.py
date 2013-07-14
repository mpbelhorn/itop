# -*- coding: utf-8 -*-
"""
A module for tracking the focal point of a spherical mirror.

"""

import time
from itop.beam.beam import Beam
from itop.beam.profiler import Alignment
from itop.utilities import save_object
from itop.utilities import load_object

class DataPoint(object):
  def __init__(self, mirror_position, beam_a, beam_b)
    self.mirror_position = mirror_position
    self.beam_a = beam_a
    self.beam_b = beam_b

  def realign(self, alignment):
    pass

  def focal_point(self):
    pass

  def translate(self, displacement):
    pass

class Instrument(object):
  """An iTOP mirror scanning instrument.

  The instrument consists of a beam tracker, a mirror fixed to a calibrated
  mirror translation stage, and a valid set of stage-beam alignment data.

  Output data is kept in order to make descisions about future movements
  relative to the mirror.

  """
  def __init__(self, tracker, mirror, alignment=None):
    self.tracker = tracker
    self.mirror = mirror
    self.alignment = None
    if alignment is not None:
      try:
        if alignment.alignment_date() is not None
          self.alignment = alignment
      except AttributeError:
        try:
          file_data = load_object(alignment)
          if file_data.alignment_date() is not None
            self.alignment = file_data
        except AttributeError:
          # Input alignment instance or file path is not valid.
          pass
    if self.alignment is None:
      self.alignment = Alignment()
      self.tracker.driver.home()
      self.alignment.align(self.tracker, home=True)

    # Output data.
    self.data = [] # (mirror_position, beam_a, beam_b)

  def sample_position(self, mirror_position, refresh=False, proximal=False):
    """Returns the reflected beam trajectories with the mirror at the given
    mirror stage position.

    Takes two optional keyword arguments.
    refresh  -- Boolean (False). If true, the trajectory data is cleared
                and the beams are relocated.
    proximal -- Boolean (False). Implies refresh. The beams are relocated
                assuming they are very near the last trajectories.

    """
    self.mirror.position(mirror_position, wait=True)
    start_point = None
    if proximal:
      try:
        start_point = self.data[-1].beam_a.first_sample() + [-25, 0, 0]
      except IndexError:
        start_point = None
    # Block beam 'B' and find beam 'A' trajectory.
    shutter = self.tracker.driver.shutter_state
    shutter(0, 0)
    shutter(1, 1)
    time.sleep(0.25)
    beam_a = self.tracker.find_beam_trajectory(start_point)
    # Block beam 'A' and find beam 'B' trajectory.
    shutter(1, 0)
    shutter(0, 1)
    time.sleep(0.25)
    beam_b = self.tracker.find_beam_trajectory(
        beam_a.last_sample() + [-20, 0, 0], scan_direction_z=-1)
    shutter(1, 1)
    data_point = DataPoint(self.mirror.position(), beam_a, beam_b)
    self.data.append(data_point)
    return data_point

  def save_data(self, path):
    # TODO - filter path name.
    output = list(self.data)
    output.insert(0, self.alignment)
    save_object(self.data, path)

