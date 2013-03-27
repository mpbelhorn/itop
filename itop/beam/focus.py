"""
A module for tracking the focal point of a spherical mirror.
"""

from itop.beam.beam import Beam
import numpy as np
from itop.math import linalg, optics

class FocalPoint(object):
  """
  Finds, calculates and returns information about the focal point
  and beam crossing positions in general of two beams.
  """
  def __init__(self, controller, group_id, camera, mirror):
    self.controller = controller
    self.mirror = self.controller.axis1
    self.group_id = group_id
    self.camera = camera
    self.mirror = mirror
    self.beam_a = Beam(self.controller, self.group_id, self.camera)
    self.beam_b = Beam(self.controller, self.group_id, self.camera)
    self.alignment = None

  def findTrajectories(self, proximal=False):
    """
    Initilizes the trajectories of both beams.

    Takes one keyword argument.
    proximal -- Boolean (False). If true, the last trajectory data is used
                to narrow the search for the new trajectory.
    """
    starting_x_coordinate = -125
    if proximal and self.beam_a.slope is not None:
      starting_x_coordinate = self.beam_a.intercepts[0][0] - 10.0
    # Block beam 'B' and find beam 'A' trajectory.
    raw_input("Clear beam 'A' and block beam 'B'. Press enter to continue.")
    self.beam_a.findTrajectory(starting_x_coordinate)
    # Block beam 'A' and find beam 'B' trajectory.
    raw_input("Clear beam 'B' and block beam 'A'. Press enter to continue.")
    self.beam_b.findTrajectory(starting_x_coordinate)

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
        self.beam_a.intercepts, self.beam_a.intercept_uncertainties,
        self.beam_b.intercepts, self.beam_b.intercept_uncertainties,
        plane=plane)

  def data(self):
    """
    Returns a list of the stage-frame-of-reference focal point data.
    """
    return [self.mirror.position(),
            self.beam_a.slope.tolist(),
            self.beam_b.slope.tolist(),
            self.focus('T'),
            self.focus('S'),
            self.radius()]

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
    if self.alignment is None:
      return None
    else:
      x_displacement = self.alignment.x_displacement
      y_displacement = self.alignment.y_displacement
      angles = self.alignment.angles
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
          self.beam_a.slope, self.alignment.angles)
      downstream_b = linalg.rotateYxzTaitBryan(
          self.beam_b.slope, self.alignment.angles)
      normal_a = optics.reconstructMirrorNormal(downstream_a)
      normal_b = optics.reconstructMirrorNormal(downstream_b)
      return [optics.radiusFromNormals(
          normal_a, normal_b, x_displacement, y_displacement),
          uncertainty]
