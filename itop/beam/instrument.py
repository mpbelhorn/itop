# -*- coding: utf-8 -*-
"""
A module for integrating iTOP mirror testing instrumentation into a single
interface for collecting iTop Mirror data.

"""

import time
from itop.motioncontrol.controller import expose_single_beam
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
  def __init__(self, tracker, mirror, alignment, input_y, tilted=False):
    self.tracker = tracker
    self.mirror = mirror
    self.tilted = tilted
    self.input_y = input_y
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
      self.tracker.devices['driver'].home()
      self.alignment.align(self.tracker, home=True, tilted=tilted,
          mirror=mirror)
      save_object(self.alignment, alignment)

    # Output data.
    self.data = []

  def sample_position(self, mirror_x, start_point=None,
      x_scan_direction=1):
    """Returns the reflected beam trajectories with the mirror at the given
    mirror stage position.

    Takes the optional keyword arguments.
      start_point (None):
        Start the scan at a position given by the coordinates in the form
        [x, y, z]. By default, the start point is determined from the
        stage limits.

      x_scan_direction (1):
        The direction to scan for the beam in x.

    """
    self.mirror.position(mirror_x, wait=True)
    tracked_beams = []
    if start_point is None:
      start_point, x_scan_direction = self._find_start_point(mirror_x)
    for beam_index, in_range in enumerate(
        tuple(self._beams_in_range(mirror_x))):
      if not in_range:
        print('Skipping Beam {}'.format(beam_index))
        tracked_beams.append(None)
        continue
      expose_single_beam(self.tracker.devices['driver'],
          beam_index,
          self.alignment.beam_count())
      time.sleep(0.25)
      tracked_beams.append(
          self.tracker.find_beam_trajectory(
              start_point,
              x_scan_direction,
              scan_direction_z=(1 if start_point[2] < 0 else -1),
              measure_power=True
              ))
      if tracked_beams[-1] is None:
        print('Failed to find beam {}.'.format(beam_index))
      start_point = self.tracker.position().array()
    self.data.append(
        DataPoint([self.mirror.position().value, self.input_y, 0],
                  tracked_beams))
    return self.data[-1]

  def save_data(self, path):
    """Saves the data in a serialized object format to a gzipped tarball at
    the given path. To reopen the data, unpickle it using
    itop.utilities.load_object()

    """
    output = list(self.data)
    output.insert(0, self.alignment)
    save_object(output, path)

  def _find_start_point(self, mirror_x):
    """Return the best starting point and scan direction for the beam
    intercept scan."""
    start_point = None
    scan_direction = 1
    first_index = self._lowest_beam_in_range(mirror_x)
    if first_index is None:
      raise InstrumentError(
          'No beam reflected (mirror position = {})'.format(mirror_x))
    last_index, start_point = self._last_beam_sample()
    if start_point is not None and last_index is not None:
      if first_index == last_index:
        if (mirror_x - self.data[-1].mirror_position[0]) < 0:
          scan_direction = -1
      elif start_point[2] > 0:
        if first_index > last_index:
          start_point = start_point - [35, 0, 0]
        else:
          start_point = start_point + [35, 0, 0]
          scan_direction = -1
      else:
        if first_index > last_index:
          start_point = start_point + [35, 0, 0]
          scan_direction = -1
        else:
          start_point = start_point - [35, 0, 0]
    else:
      start_point = [self.tracker.axes[0].limits.lower,
                     self.tracker.axes[1].limits.lower,
                     self.tracker.axes[2].limits.lower]
    return (start_point, scan_direction)

  def _beams_in_range(self, mirror_x):
    """Return a generator of booleans indicating that the ith indexed beam is
    in range at the given mirror position."""
    if not self.tilted:
      for index in self.alignment.beam_indexes():
        yield not self.alignment.out_of_range(index, [mirror_x, 0, 0])
    else:
      for index in self.alignment.beam_indexes():
        yield True

  def _lowest_beam_in_range(self, mirror_x):
    """Return the lowest beam index that is in range or None if no beams are
    in range."""
    return next(
        (i for i, j in enumerate(self._beams_in_range(mirror_x)) if j),
        None)
  def _last_beam_samples(self):
    """Return a generator of the last beam sample for each beam. None is used
    if a beam was not visible in the last sampling.
    """
    try:
      for index, beam in enumerate(self.data[-1].beams):
        yield (index, beam.last_sample() if beam else None)
    except (IndexError, AttributeError):
      # Last data point has no beams.
      yield (None, None)

  def _last_beam_sample(self):
    """Return the (index, position) of the last beam sample in data. If no
    beam has been sampled, return (None, None)."""
    return next(
        (i for i in reversed(tuple(self._last_beam_samples())) if i[1]),
        (None, None))
