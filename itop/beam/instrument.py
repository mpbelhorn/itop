# -*- coding: utf-8 -*-
"""
A module for integrating iTOP mirror testing instrumentation into a single
interface for collecting iTop Mirror data.

"""

import time
from itop.beam.datapoint import DataPoint
from itop.beam.alignment import Alignment
from itop.utilities import save_object
from itop.utilities import load_object


class InstrumentError(Exception):
  """Exception and error handling for the instrument class."""
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
        if alignment.alignment_date() is not None:
          self.alignment = alignment
      except AttributeError:
        try:
          file_data = load_object(alignment)
          if file_data.alignment_date() is not None:
            self.alignment = file_data
        except (AttributeError, IOError):
          # Input alignment file is not valid.
          pass
    if self.alignment is None:
      self.alignment = Alignment()
      self.tracker.driver.home()
      self.alignment.align(self.tracker, home=True)
      save_object(self.alignment, alignment)

    # Output data.
    self.data = [] # (mirror_position, beam_a, beam_b)

  def sample_position(self, mirror_position, proximal=False, start_point=None):
    """Returns the reflected beam trajectories with the mirror at the given
    mirror stage position.

    Takes the optional keyword arguments.
      start_point (None):
        Start the scan at a position given by the coordinates in the form
        [x, y, z]. By default, the start point is determined from the
        stage limits.

      proximal (False):
        The scan start point is automatically determined assuming the beam
        trajectories are very near their last known trajectories.

    """
    self.mirror.position(mirror_position, wait=True)
    if proximal:
      try:
        start_point = self.data[-1].beam_a.first_sample() + [-25, 0, 0]
      except IndexError:
        # No established trajectories.
        pass
    if start_point is None:
      start_point = [self.tracker.axes[0].limits.lower,
                     self.tracker.axes[1].limits.lower,
                     self.tracker.axes[2].limits.lower]
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
    """Saves the data in a serialized object format to a gzipped tarball at
    the given path. To reopen the data, unpickle it using
    itop.utilities.load_object()

    """
    output = list(self.data)
    output.insert(0, self.alignment)
    save_object(output, path)

