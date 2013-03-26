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
  def __init__(self, controller, group_id, camera, mirror, **kwargs):
    self.controller = controller
    self.mirror = self.controller.axis1
    self.group_id = group_id
    self.camera = camera
    self.mirror = mirror

    self.beam_a = Beam(self.controller, self.group_id, self.camera)
    self.beam_b = Beam(self.controller, self.group_id, self.camera)
    self.alignment = None
    self.tangential_focus_a = None
    self.tangential_focus_b = None
    self.sagittal_focus_a = None
    self.sagittal_focus_b = None

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
    # Find the tangential focal plane.
    # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
    slope_a = self.beam_a.slope
    upstream_a = self.beam_a.intercepts[0]
    slope_b = self.beam_b.slope
    upstream_b = self.beam_b.intercepts[0]
    tangential_coefficients = np.array([
        [slope_a[2], -slope_a[0]], [slope_b[2], -slope_b[0]]])
    tangential_ordinates = np.array([
        slope_a[2] * upstream_a[0] - slope_a[0] * upstream_a[2],
        slope_b[2] * upstream_b[0] - slope_b[0] * upstream_b[2]])
    tangential_solution = np.linalg.solve(
        tangential_coefficients, tangential_ordinates)
    # Find the sagittal focal plane.
    sagittal_coefficients = np.array([
        [slope_a[2], -slope_a[1]], [slope_b[2], -slope_b[1]]])
    sagittal_ordinates = np.array([
        slope_a[2] * upstream_a[1] - slope_a[1] * upstream_a[2],
        slope_b[2] * upstream_b[1] - slope_b[1] * upstream_b[2]])
    sagittal_solution = np.linalg.solve(
        sagittal_coefficients, sagittal_ordinates)
    # Find the circle of least confusion.
    #   This will have to wait until the camera can be remotely
    #   positioned vertically. Until then, the vertical spot width data
    #   needed to locate the CoLC is unreliable.
    beam_a_tangential_fraction = (
        (tangential_solution[1] - upstream_a[2]) / slope_a[2])
    beam_b_tangential_fraction = (
        (tangential_solution[1] - upstream_b[2]) / slope_b[2])
    self.tangential_focus_a = self.beam_a.position(beam_a_tangential_fraction)
    self.tangential_focus_b = self.beam_b.position(beam_b_tangential_fraction)
    beam_a_sagittal_fraction = (
        (sagittal_solution[1] - upstream_a[2]) / slope_a[2])
    beam_b_sagittal_fraction = (
        (sagittal_solution[1] - upstream_b[2]) / slope_b[2])
    self.sagittal_focus_a = self.beam_a.position(beam_a_sagittal_fraction)
    self.sagittal_focus_b = self.beam_b.position(beam_b_sagittal_fraction)
    # TODO - If stage can move to focal point, verify the position is correct.
    return self.data()

  def focus(self, intercepts_a, intercepts_b, plane='T'):
    """
    Returns the focal point for the given beam intercepts.

    The beam intercepts are lists of the form (p_upstream, p_downstream)
    where the p_upstream and p_downstream are (x,y,z) points on the beam
    in the specified relative positions.

    Takes the following optional keyword argument:
    plane  --  Selects the (T)angential or (S)agittal focal plane.
               Accpetable values are 'T' (default) or 'S'
    """
    index = 1 if plane is 'S' else 0
    sa = intercepts_a[1] - intercepts_a[0]
    sb = intercepts_b[1] - intercepts_b[0]
    coefficients = np.array([[sa[2], -sa[index]], [sb[2], -sb[index]]])
    ordinates = np.array([sa[2] * intercepts_a[index] - sa[index] * intercepts_a[2],
                          sb[2] * intercepts_b[index] - sb[index] * intercepts_b[2]])
    solution = np.linalg.solve(coefficients, ordinates)
    fraction_a = ((solution[1] - intercepts_a[2]) / sa[2])
    fraction_b = ((solution[1] - intercepts_b[2]) / sb[2])
    focus_a = (intercepts_a + fraction_a * sa)
    focus_b = (intercepts_b + fraction_b * sb)
    return (focus_a, focus_b)

  def foci(self, intercepts_a, uncertainties_a,
      intercepts_b, uncertainties_b, plane='T'):
    """
    Returns the focal point and its uncertainty for the given beam intercepts.

    The beam intercepts are lists of the form (p_upstream, p_downstream)
    where the p_upstream and p_downstream are (x,y,z) points on the beam
    in the specified relative positions.

    Takes the following optional keyword argument:
    plane  --  Selects the (T)angential or (S)agittal focal plane.
               Accpetable values are 'T' (default) or 'S'
    """
    index = 1 if plane is 'S' else 0
    foci = []
    limits = []
    signs_list = [[ 0, 0],
                  [ 1,-1],
                  [-1, 1]]
    for signs in signs_list:
      offset = np.array([0, 0, 0])
      ri_a = intercepts_a[0] + signs[0] * (offset + np.array(uncertainties_a[0][index]))
      rf_a = intercepts_a[1] + signs[0] * (offset + np.array(uncertainties_a[1][index]))
      ri_b = intercepts_b[0] + signs[1] * (offset + np.array(uncertainties_b[0][index]))
      rf_b = intercepts_b[1] + signs[1] * (offset + np.array(uncertainties_b[1][index]))
      foci.append(self.focus([ri_a, rf_a], [ri_b, rf_b], plane=plane))
    return [foci[0], [foci[1], foci[2]]]

  def data(self):
    """
    Returns a list of the stage-frame-of-reference focal point data.
    """
    return [self.mirror.position(),
            [self.beam_a.slope.tolist()],
            [self.beam_b.slope.tolist()],
            self.tangential_focus_a,
            self.tangential_focus_b,
            self.sagittal_focus_a,
            self.sagittal_focus_b,
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
      # TODO - Calculate uncertainty.
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
