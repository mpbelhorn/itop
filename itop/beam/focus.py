"""
A module for tracking the focal point of a spherical mirror.
"""

from itop.beam.beam import Beam
import numpy as np
import itop.math as im

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
    self.tangential_focus_a = None
    self.tangential_focus_b = None
    self.sagittal_focus_a = None
    self.sagittal_focus_b = None

  def findTrajectories(self):
    """
    Initilizes the trajectories of both beams.
    """
    # Block beam 'B' and find beam 'A' trajectory.
    raw_input("Clear(Block) beam 'A'('B'). Press enter to continue.")
    self.beam_a.findTrajectory()
    # Block beam 'A' and find beam 'B' trajectory.
    raw_input("Clear(Block) beam 'B'('A'). Press enter to continue.")
    self.beam_b.findTrajectory()

  def findFocalPoints(self, refresh=False):
    """
    Finds the focal points (assuming astigmatism) of the beams.
    """
    # Initialize the beam trajectories if necessary.
    if ((self.beam_a.slope is None) or
        (self.beam_b.slope is None) or
        refresh):
      self.findTrajectories()
    else:
      print "Trajectories already initialized."
    # Find the tangential focal plane.
    # Equations in the form (sz*x - sx*z == sz*x0 - sx*z0)
    slope_a = self.beam_a.slope
    upstream_a = self.beam_a.r_initial
    slope_b = self.beam_b.slope
    upstream_b = self.beam_b.r_initial
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

  def data(self):
    """
    Returns a list of the focal point data.
    """
    return [self.mirror.position(),
            self.beam_a.slope,
            self.beam_b.slope,
            self.tangential_focus_a,
            self.tangential_focus_b,
            self.sagittal_focus_a,
            self.sagittal_focus_b]

  def radius(self,
      x_displacement, y_displacement,
      correction_about_y, correction_about_x):
    """
    Returns the radius of curvature after correcting for the rotated camera
    coordinate system with the given correction angles (radians).
    """
    downstream_a = im.linalg.normalize(
        im.linalg.rotateVector(
            im.linalg.rotateVector(
                self.beam_a.slope, correction_about_x, [1,0,0]),
            correction_about_y, [0,1,0]))
    downstream_b = im.linalg.normalize(
        im.linalg.rotateVector(
            im.linalg.rotateVector(
                self.beam_b.slope, correction_about_x, [1,0,0]),
            correction_about_y, [0,1,0]))
    normal_a = im.optics.reconstructMirrorNormal(downstream_a)
    normal_b = im.optics.reconstructMirrorNormal(downstream_b)
    return im.optics.radiusFromNormals(
        normal_a, normal_b, x_displacement, y_displacement)
