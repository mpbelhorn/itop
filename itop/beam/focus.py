# -*- coding: utf-8 -*-
"""
A module for tracking the focal point of a spherical mirror.
"""

from itop.beam.beam import Beam
import numpy as np
from itop.math import linalg, optics
from collections import namedtuple

FocusData = namedtuple('FocusData',
    ['mirror_position',
     'beam_a_trajectory',
     'beam_b_trajectory',
     'tangential_focus',
     'sagittal_focus',
     'mirror_radius'])

class FocalPoint(object):
  """
  Finds, calculates and returns information about the focal point
  and beam crossing positions in general of two beams.
  """
  def __init__(self, tracker, mirror):
    self.tracker = tracker
    self.mirror = mirror
    self.beam_a = Beam(self.tracker)
    self.beam_b = Beam(self.tracker)
    self.alignment = None

  def findTrajectories(self, proximal=False):
    """
    Initilizes the trajectories of both beams.

    Takes one keyword argument.
    proximal -- Boolean (False). If true, the last trajectory data is used
                to narrow the search for the new trajectory.
    """
    start_point = [-125., 20., -125.]
    if proximal and self.beam_a.slope is not None:
      start_point = self.beam_a.upstream_point + np.array([-25, 0, 0])
    # Block beam 'B' and find beam 'A' trajectory.
    shutter = self.tracker.driver.shutterState
    shutter(0, 0)
    shutter(1, 1)
    self.beam_a.findTrajectory(*start_point)
    # Block beam 'A' and find beam 'B' trajectory.
    shutter(1, 0)
    shutter(0, 1)
    self.beam_b.findTrajectory(
        *((self.beam_a.downstream_point +
        np.array([-20, 0, 0])).tolist()), scan_direction_z=-1)
    shutter(1, 1)


  def findFocalPoints(self, mirror_position, refresh=False, proximal=False):
    """
    Finds the focal points (assuming astigmatism) of the beams.

    Takes two optional keyword arguments.
    refresh  -- Boolean (False). If true, the trajectory data is cleared
                and the beams are relocated.
    proximal -- Boolean (False). Implies refresh. The beams are relocated
                assuming they are very near the last trajectories.
    """
    self.mirror.position(mirror_position, wait=True)

    # Initialize the beam trajectories if necessary.
    if proximal:
      self.findTrajectories(proximal=True)
    elif ((self.beam_a.slope is None) or
        (self.beam_b.slope is None) or
        refresh):
      self.findTrajectories()
    else:
      print "Trajectories already initialized."
    return self.data()

  def focus(self, plane='T'):
    """
    Returns the focal point with uncertainties for the given plane.

    Takes the following optional keyword argument:
    plane  --  Selects the (T)angential or (S)agittal focal plane.
               Accpetable values are 'T' (default) or 'S'
    """
    return optics.focusWithUncertainty(
        self.beam_a.trajectory(),
        self.beam_b.trajectory(),
        plane=plane)

  def data(self):
    """
    Returns a list of the stage-frame-of-reference focal point data.
    """
    return FocusData(self.mirror.position(),
                     self.beam_a.trajectory(),
                     self.beam_b.trajectory(),
                     self.focus('T'),
                     self.focus('S'),
                     self.radius())

  def radius(self):
    """
    Return the radius of curvature of a mirror based on the relative change
    in direction of two reflected parallel beams with known spatial
    displacement.

    The beams are taken to travel in the -z_beam direction. Thus outgoing
    slope coordinates must be rotated into the beam frame by applying a
    rotation of phi about the y-axis followed by a rotation of theta about the
    x'-axis. phi and theta are the first and second elements in the list
    itop.beam.alignment.BeamAlignment.angles.
    """
    if self.tracker.alignment is None:
      return None
    else:
      alignment = self.tracker.alignment
      x_displacement = alignment.x_displacement
      y_displacement = alignment.y_displacement
      angles = alignment.angles
      alternates = []
      for i in self.beam_a.slopeUncertainty():
        for j in self.beam_b.slopeUncertainty():
          da = linalg.rotateYxzTaitBryan(i, angles)
          db = linalg.rotateYxzTaitBryan(j, angles)
          na = optics.reconstructMirrorNormal(da)
          nb = optics.reconstructMirrorNormal(db)
          alternates.append(
              optics.radiusFromNormals(na, nb, x_displacement, y_displacement))
      uncertainty = np.std(alternates)
      downstream_a = linalg.rotateYxzTaitBryan(
          self.beam_a.slope, alignment.angles)
      downstream_b = linalg.rotateYxzTaitBryan(
          self.beam_b.slope, alignment.angles)
      normal_a = optics.reconstructMirrorNormal(downstream_a)
      normal_b = optics.reconstructMirrorNormal(downstream_b)
      return [optics.radiusFromNormals(
          normal_a, normal_b, x_displacement, y_displacement),
          uncertainty]
