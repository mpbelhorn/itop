"""
A package for manipulating itop mirror testing data and generating analysis
plots.
"""

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from itop.math import Vector
import numpy as np
from itop.math.linalg import rotation_matrix
from itop.math.optics import focus, radius_from_normals as radius
from itop.math.optics import reconstruct_mirror_normal as mirror_normal


def distortions(data):
  """Returns a pair ([d_a], [d_b]) where [d_x] is a list of the
  beam distortion of beam_x."""
  print data[0]

def input_positions(
    mirror_center_from_calibration, beam_separation, alignment):
  """Computes the beam input position as a vector that lies in the input face
  of the mirror.

  The displacement of the mirror center from the calibration point should
  include mirror height. The beam separation should be given as pb - pa in the
  lab YX plane at the calibration point.

  The vector is expressed in a coordinate system that is parallel to the table
  system but with an origin that is fixed to the mirror-frame origin.

  """
  nominal_in_a = Vector(mirror_center_from_calibration)
  nominal_in_a[1] = -nominal_in_a[1]
  nominal_in_b = nominal_in_a - beam_separation
  beam_a_true_input = (
      nominal_in_a.dot(alignment.beam_a.direction) *
      Vector(alignment.beam_a.direction) - nominal_in_a)
  beam_b_true_input = (
      nominal_in_b.dot(alignment.beam_b.direction) *
      Vector(alignment.beam_b.direction) - nominal_in_b)
  return (beam_a_true_input, beam_b_true_input)

def translate_beams(data_point, displacement):
  """Return the pair of beams in a DataPoint translated by the given
  displacement vector.
  """
  return (data_point.beam_a.translate(displacement),
          data_point.beam_b.translate(displacement))

def align_data_in_mirror_frame(data, alignment, mirror_height, calibration):
  """Return the data transformed into the mirror-centered frame.

  The following displacements are compensated for:
    1.) The x,y,z displacement between the tracker calibration point (tcal)
        and the mirror stage calibration point (mcal).
    2.) The relative displacement of the tracker origin and the mirror
        center for a given mirror stage position.
    3.) The true beam input position assuming the tracker and mirror
        stages are perfectly parallel and the mirror is rotated
        w.r.t. the lab frame according to the alignment data.

  The calibration should be an itop.Vector with the correct signs and
  uncertainties for the displacement

      Vector(tcal) - Vector(mcal)

  in the tracker/lab frame.
  """
  tcal_from_tracker = Vector(
      [125.0, 0.0, -125.0], [[0.0005, 0.0075, 0.0005]])
  tracker_from_mcal = Vector(calibration) - tcal_from_tracker

  beam_separation = Vector(
      alignment.beam_b.position(-calibration[2] - tcal_from_tracker[2]) -
      alignment.beam_a.position(-calibration[2] - tcal_from_tracker[2]))

  matrix = rotation_matrix(-alignment.beam_a.direction)
  output = []
  for i in data:
    mirror_from_mcal = Vector([i.mirror_position, mirror_height, 0])
    tracker_from_mirror = tracker_from_mcal - mirror_from_mcal

    beams = translate_beams(i, tracker_from_mirror)
    inputs = input_positions(mirror_from_mcal, beam_separation, alignment)
    beams = [i.transform(matrix) for i in beams]
    inputs = [i.transform(matrix) for i in inputs]
    output.append(((inputs[0], beams[0]), (inputs[1], beams[1])))
  return output

def focii(data, beam_1, beam_2, plane):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]
  and return a list of the focal points for beam_1 with beam_2 in the
  'T'angential or 'S'agittal plane. The beam_n are indexes 0 for beam_a or
  1 for beam_b. The output list is in the format:
    ((beam_1_input, beam_1_focus), (beam_2_input, beam_2_focus))
  """
  output = []
  for i in data:
    for j in data:
      if beam_1 == beam_2 and i is j:
        pass
      else:
        focal_point = focus(i[beam_1][1], j[beam_2][1], plane=plane)
        output.append(
            ((i[beam_1][0], focal_point[0]), (j[beam_2][0], focal_point[1])))
  return output

def radii(data, beam_1, beam_2, **kwargs):
  """Return a list of the radii between beam_1 and beam_2 in the given data
  as a tuple (s, r) where s is the absolute separation distance in x and r
  is the radius. Any additional keyword arguments are passed on to the
  mirror normal reconstruction.
  """
  if beam_1 == beam_2:
    return [(i[beam_1][0].array()[0] - j[beam_2][0].array()[0],
             radius(mirror_normal(i[beam_1][1].direction.array(), **kwargs),
                    mirror_normal(j[beam_2][1].direction.array(), **kwargs),
                    i[beam_1][0].array()[:2],
                    j[beam_2][0].array()[:2])
            ) for i in data for j in data if i is not j]
  else:
    return [(i[beam_1][0].array()[0] - j[beam_2][0].array()[0],
             radius(mirror_normal(i[beam_1][1].direction.array(), **kwargs),
                    mirror_normal(j[beam_2][1].direction.array(), **kwargs),
                    i[beam_1][0].array()[:2],
                    j[beam_2][0].array()[:2])
            ) for i in data for j in data]

STYLE = {
    'aat_color': '#ff0000',
    'abt_color': '#ff6600',
    'bat_color': '#ff9900',
    'bbt_color': '#ffff00',
    'aas_color': '#0066ff',
    'abs_color': '#0000ff',
    'bas_color': '#00ffff',
    'bbs_color': '#0099ff',
    'labelsize': 14,
    'titlesize': 14}

def draw_radii(data, **kwargs):
  """Plot the radius of the mirror as a function of beam separation distance
  in x. The radius is computed from each of the beam pairs in the given data.
  Data points must be in the form of
      ((beam_a_input, beam_a), (beam_b_input, beam_b))).

  Any keyword arguments are passed on to the mirror normal reconstruction
  method.
  """
  aa_radii = radii(data, 0, 0, **kwargs)
  ab_radii = radii(data, 0, 1, **kwargs)
  bb_radii = radii(data, 1, 1, **kwargs)
  plt.figure(figsize=(20, 4), dpi=150)
  plt.suptitle("Calculated Radius vs Beam Separation",
      fontsize=STYLE['titlesize'])
  plt.subplot(131)
  plt.plot(*zip(*aa_radii), marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75, label="A-A Radii")
  plt.subplot(132)
  plt.plot(*zip(*ab_radii), marker='o', ls='None',
      color=STYLE['abt_color'], alpha=0.75, label="A-B Radii")
  plt.subplot(133)
  plt.plot(*zip(*bb_radii), marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75, label="B-B Radii")
  plt.legend(loc='best', numpoints=1)
  plt.ylabel("Radius [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Beam Separation [mm]", fontsize=STYLE['labelsize'])
  plt.show()

def draw_focii_vs_input_tangential(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the tangential focii against the beam-mirror input positions.
  """
  beam_0_focii = np.array(focii(data, 0, 0, 'T'))
  beam_1_focii = np.array(focii(data, 1, 1, 'T'))
  b0in = [i.value for i in beam_0_focii[:, 0, 0, 0]]
  b0x, b0y, b0z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_0_focii[:,0,1]])
  b1in = [i.value for i in beam_1_focii[:, 0, 0, 0]]
  b1x, b1y, b1z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_1_focii[:,0,1]])

  plt.figure(1, figsize=(18, 4), dpi=150)
  plt.suptitle("Tangential Focus Coordinates vs Beam Input Position",
      fontsize=STYLE['titlesize'])
  plt.subplot(1, 3, 1)
  plt.plot(b0in, b0x, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75, label="Tangential A-A")
  plt.plot(b1in, b1x, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75, label="Tangential B-B")
  plt.legend(loc='best', numpoints=1)
  plt.ylabel("Focus X coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 2)
  plt.plot(b0in, b0y, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75)
  plt.plot(b1in, b1y, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75)
  plt.ylabel("Focus Y coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 3)
  plt.plot(b0in, b0z, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75)
  plt.plot(b1in, b1z, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75)
  plt.ylabel("Focus Z coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.show()

def draw_focii_vs_input_sagittal(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the tangential focii against the beam-mirror input positions.
  """
  beam_focii = np.array(focii(data, 0, 1, 'S'))
  b0in = [i.value for i in beam_focii[:, 0, 0, 0]]
  b0x, b0y, b0z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_focii[:,0,1]])
  b1in = [i.value for i in beam_focii[:, 1, 0, 0]]
  b1x, b1y, b1z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_focii[:,1,1]])

  plt.figure(1, figsize=(18, 4), dpi=150)
  plt.suptitle("Sagittal Focus Coordinates vs Beam Input Position",
      fontsize=STYLE['titlesize'])
  plt.subplot(1, 3, 1)
  plt.plot(b0in, b0x, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75, label="Sagittal A-A")
  plt.plot(b1in, b1x, marker='o', ls='None',
      color=STYLE['bas_color'], alpha=0.75, label="Sagittal B-B")
  plt.legend(loc='best', numpoints=1)
  plt.ylabel("Focus X coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 2)
  plt.plot(b0in, b0y, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75)
  plt.plot(b1in, b1y, marker='o', ls='None',
      color=STYLE['bas_color'], alpha=0.75)
  plt.ylabel("Focus Y coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 3)
  plt.plot(b0in, b0z, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75)
  plt.plot(b1in, b1z, marker='o', ls='None',
      color=STYLE['bas_color'], alpha=0.75)
  plt.ylabel("Focus Z coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.show()

def draw_focii_in_space_tangential(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the tangential focii in 3D space projections in a mirror-centered
  frame.
  """
  beam_0_focii = np.array(focii(data, 0, 0, 'T'))
  beam_1_focii = np.array(focii(data, 1, 1, 'T'))
  b0x, b0y, b0z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_0_focii[:,0,1]])
  b1x, b1y, b1z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_1_focii[:,0,1]])

  fig = plt.figure(1, figsize=(14, 9), dpi=150)
  zy_plot = plt.subplot(223)
  plt.plot(b0z, b0y, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'], label='Tangential A-A')
  plt.plot(b1z, b1y, marker='o', ls='None',
      alpha=0.7, color=STYLE['bbt_color'], label='Tangential B-B')
  plt.legend(loc='best', numpoints=1)
  zx_plot = plt.subplot(221, sharex=zy_plot)
  plt.plot(b0z, b0x, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'])
  plt.plot(b1z, b1x, marker='o', ls='None',
      alpha=0.7, color=STYLE['bbt_color'])
  xy_plot = plt.subplot(224, sharey=zy_plot)
  plt.plot(b0x, b0y, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'])
  plt.plot(b1x, b1y, marker='o', ls='None',
      alpha=0.7, color=STYLE['bbt_color'])
  for xlabel_i in zx_plot.axes.get_xticklabels():
    xlabel_i.set_visible(False)
  for ylabel_i in xy_plot.axes.get_yticklabels():
    ylabel_i.set_visible(False)
  zy_plot.set_xlabel('z Coordinate [mm]', fontsize=STYLE['labelsize'])
  zy_plot.xaxis.set_major_locator(MaxNLocator(14, prune='both'))
  zy_plot.set_ylabel('y Coordinate [mm]', fontsize=STYLE['labelsize'])
  zy_plot.yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  xy_plot.set_title('xy-Plane', fontsize=STYLE['titlesize'])
  xy_plot.set_xlabel('x Coordinate [mm]', fontsize=STYLE['labelsize'])
  xy_plot.xaxis.set_major_locator(MaxNLocator(9, prune='both'))
  zx_plot.set_title('zx-Plane', fontsize=STYLE['titlesize'])
  zx_plot.set_ylabel('x Coordinate [mm]', fontsize=STYLE['labelsize'])
  zx_plot.yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  fig.subplots_adjust(hspace=0, wspace=0)
  fig.suptitle('Focal Points in Mirror Coordinate System',
       fontsize=STYLE['titlesize'], fontweight='bold')

def draw_focii_in_space_sagittal(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the sagittal focii in 3D space projections in a mirror-centered
  frame.
  """
  beam_focii = np.array(focii(data, 0, 1, 'S'))
  b0x, b0y, b0z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_focii[:,0,1]])
  b1x, b1y, b1z = zip(
      *[(i[0].value, i[1].value, i[2].value) for i in beam_focii[:,1,1]])

  fig = plt.figure(2, figsize=(14, 9), dpi=150)
  zy_plot = plt.subplot(223)
  plt.plot(b0z, b0y, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'], label='Sagittal A-B')
  plt.plot(b1z, b1y, marker='o', ls='None',
      alpha=0.7, color=STYLE['bas_color'], label='Sagittal B-A')
  plt.legend(loc='best', numpoints=1)
  zx_plot = plt.subplot(221, sharex=zy_plot)
  plt.plot(b0z, b0x, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'])
  plt.plot(b1z, b1x, marker='o', ls='None',
      alpha=0.7, color=STYLE['bas_color'])
  xy_plot = plt.subplot(224, sharey=zy_plot)
  plt.plot(b0x, b0y, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'])
  plt.plot(b1x, b1y, marker='o', ls='None',
      alpha=0.7, color=STYLE['bas_color'])
  for xlabel_i in zx_plot.axes.get_xticklabels():
    xlabel_i.set_visible(False)
  for ylabel_i in xy_plot.axes.get_yticklabels():
    ylabel_i.set_visible(False)
  zy_plot.set_xlabel('z Coordinate [mm]', fontsize=STYLE['labelsize'])
  zy_plot.xaxis.set_major_locator(MaxNLocator(14, prune='both'))
  zy_plot.set_ylabel('y Coordinate [mm]', fontsize=STYLE['labelsize'])
  zy_plot.yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  xy_plot.set_title('xy-Plane', fontsize=STYLE['titlesize'])
  xy_plot.set_xlabel('x Coordinate [mm]', fontsize=STYLE['labelsize'])
  xy_plot.xaxis.set_major_locator(MaxNLocator(9, prune='both'))
  zx_plot.set_title('zx-Plane', fontsize=STYLE['titlesize'])
  zx_plot.set_ylabel('x Coordinate [mm]', fontsize=STYLE['labelsize'])
  zx_plot.yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  fig.subplots_adjust(hspace=0, wspace=0)
  fig.suptitle('Focal Points in Mirror Coordinate System',
       fontsize=STYLE['titlesize'], fontweight='bold')
