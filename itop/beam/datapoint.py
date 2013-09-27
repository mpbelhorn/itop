# -*- coding: utf-8 -*-
"""
A module for representing the data collected by the iTOP testing
instrumentation.

"""


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
        len(self.beams), self.mirror_position)

  def realign(self, alignment):
    """Applies an alignment to the data point trajectories."""
    pass

  def focal_point(self):
    """Returns the focal point of the beams."""
    pass

  def translate(self, displacement):
    """Translates the beams' data points by the given displacement vector.

    """
    pass

