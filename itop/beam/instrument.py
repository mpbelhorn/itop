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

  def sample_position(self, mirror_position, proximal=False,
      start_point=None, x_scan_direction=1):
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

      x_scan_direction (1):
        The direction to scan for the beam in x.

    """
    shutter = self.tracker.driver.shutter_state
    self.mirror.position(mirror_position, wait=True)
    tracked_beams = []
    beam_in_range = [not self.alignment.out_of_range(
      beam_index, [mirror_position, 0, 0]) for beam_index in (0, 1)]
    first_index = next(
        (i for i, j in enumerate(beam_in_range) if j), None)
    try:
      if start_point is None:
        last_beams = self.data[-1].beams
        last_samples = [(index, (beam.last_sample() if beam else None))
            for index, beam in enumerate(last_beams)]
        last_samples.reverse()
        start_index, start_point = next(
            (i for i in last_samples if i[1]), (None, None))
        dx = mirror_position - self.data[-1].mirror_position
        if first_index == start_index:
          x_scan_direction = -1 if dx < 0 else 1
        elif start_point[2] > 0:
          if first_index > start_index:
            start_point = start_point - [25, 0, 0]
            x_scan_direction = 1
          else:
            start_point = start_point + [25, 0, 0]
            x_scan_direction = -1
        else:
          if first_index > start_index:
            start_point = start_point + [25, 0, 0]
            x_scan_direction = -1
          else:
            start_point = start_point - [25, 0, 0]
            x_scan_direction = 1
    except (IndexError, AttributeError):
      pass
    if start_point is None:
      start_point = [self.tracker.axes[0].limits.lower,
                     self.tracker.axes[1].limits.lower,
                     self.tracker.axes[2].limits.lower]
      x_scan_direction = 1
    for beam_index in (0, 1):
      if not self.alignment.out_of_range(beam_index, [mirror_position, 0, 0]):
        for shutter_id in (0, 1):
          shutter(shutter_id, 0)
        shutter(beam_index, 1)
        time.sleep(0.25)
        tracked_beams.append(
            self.tracker.find_beam_trajectory(
                start_point,
                x_scan_direction,
                scan_direction_z=(1 if start_point[2] < 0 else -1)
                ))
        start_point = self.tracker.position().array()
      else:
        print('Skipping Beam {}'.format(beam_index))
        tracked_beams.append(None)
    self.data.append(DataPoint(self.mirror.position(), tracked_beams))
    return self.data[-1]

  def save_data(self, path):
    """Saves the data in a serialized object format to a gzipped tarball at
    the given path. To reopen the data, unpickle it using
    itop.utilities.load_object()

    """
    output = list(self.data)
    output.insert(0, self.alignment)
    save_object(output, path)

