# -*- coding: utf-8 -*-
"""
A module for representing the data collected by the iTOP testing
instrumentation.

"""
from itop.math.linalg import rotation_matrix


class DataPoint(object):
  """A representation of a pair of beams sampled from a given mirror
  position.

  """
  def __init__(self, mirror_position, beams):
    """Constructor for DataPoint. Takes a mirror position and two fully
    established Beam objects.

    """
    self.mirror_position = mirror_position
    self.beams = beams

  def __repr__(self):
    return "DataPoint({} @ {})".format(
        len(filter(None, self.beams)), self.mirror_position)

  def realign(self, alignment):
    """Applies an alignment to the data point trajectories."""
    offset = alignment.calibration.displacement() - self.mirror_position
    matrix = rotation_matrix(-alignment.beams[0].direction)
    new_beams = []
    for beam in self.beams:
      if beam is None:
        new_beams.append(None)
      else:
        new_beams.append((beam.translate(offset)).transform(matrix))
    return DataPoint(self.mirror_position, new_beams)

