# -*- coding: utf-8 -*-
"""
A module for checking the beam alignment by direct imaging.
"""

from itop.beam.beam import Beam
import numpy as np
import cPickle
import zlib

class BeamAlignment(object):
  """
  For establishing alignment and positioning of LBP with respect to beams.
  """
  def __init__(self, tracker):
    """
    Initialize alignment.
    """
    self.tracker = tracker
    self.beam_a = Beam(self.tracker)
    self.beam_b = Beam(self.tracker)
    self.angles = [None, None, None]
    self.x_displacement = None
    self.y_displacement = None

  def findTrajectories(self):
    """
    Initilizes the trajectories of both upstream beams.
    """
    # Rotate camera to face splitter output.
    self.tracker.facing_z_direction = 1
    # Block beam 'B' and find beam 'A' trajectory.
    shutter = self.tracker.driver.shutterState
    shutter(0, 0)
    shutter(1, 1)
    self.beam_a.findTrajectory(125, 12, 125, -1, -1)
    # Block beam 'A' and find beam 'B' trajectory.
    shutter(1, 0)
    shutter(0, 1)
    self.beam_b.findTrajectory(125-50, 2, 125, -1, -1)
    self.x_displacement, self.y_displacement = self.displacements()
    self.angles = self.beamStageAngles()
    # Rotate camera to face mirror.
    self.tracker.facing_z_direction = -1
    shutter(1, 1)

  def displacements(self):
    """
    Calculates beam displacements given the trajectories and camera offset.
    """
    x_displacement, y_displacement = (
        self.beam_b.upstream_point - self.beam_a.upstream_point)[:2]
    return (x_displacement, y_displacement)

  def beamStageAngles(self):
    """
    Returns the yxz-convention Euler angles to rotate the stage coordinate
    system into the beam coordinate system.
    """
    return [angle for angle in self.beam_a.angles()]

  def save(self, file_path):
    """
    Saves the beam alignment data to a gzipped serialized object file.
    """
    data = {}
    data['angles'] = self.angles
    data['x_displacement'] = self.x_displacement
    data['y_displacement'] = self.y_displacement
    data['beam_a'] = self.beam_a.trajectory()
    data['beam_b'] = self.beam_b.trajectory()

    with open(file_path, 'wb') as output_file:
      output_file.write(zlib.compress(
          cPickle.dumps(data, cPickle.HIGHEST_PROTOCOL),9))

  def load(self, file_path):
    """
    Loads the beam alignment data from a gzipped serialized object file.
    """
    with open(file_path, 'rb') as input_file:
      pickled_data = zlib.decompress(input_file.read())
      data = cPickle.loads(pickled_data)
      self.angles = data['angles']
      self.x_displacement = data['x_displacement']
      self.y_displacement = data['y_displacement']
      self.beam_a.load(data.get('beam_a'))
      self.beam_b.load(data.get('beam_b'))
