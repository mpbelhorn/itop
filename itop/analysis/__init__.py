"""
A package for manipulating itop mirror testing data and generating analysis
plots.
"""

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from itop.math import Vector
import numpy as np
from itop.math.linalg import rotation_matrix
from itop.math.optics import focus


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
      [121.11, 0.0, -125.315], [[0.01, 0.0075, 0.01]])
  tracker_from_mcal = Vector(calibration) - tcal_from_tracker

  beam_separation = Vector(
      alignment.beam_b.position(-calibration[2] - tcal_from_tracker[2]) -
      alignment.beam_a.position(-calibration[2] - tcal_from_tracker[2]))

  matrix = rotation_matrix(-alignment.beam_a.direction)
  output = []
  for i in data:
    mirror_from_mcal = (Vector([i.mirror_position, mirror_height, 0]) +
        Vector([39.18, 0, 0], [[0.01, 0, 0]]))
    tracker_from_mirror = tracker_from_mcal - mirror_from_mcal

    beam_a, beam_b = translate_beams(i, tracker_from_mirror)
    beam_a_input, beam_b_input = input_positions(
        mirror_from_mcal, beam_separation, alignment)
    beam_a = beam_a.transform(matrix)
    beam_b = beam_b.transform(matrix)
    beam_a_input = beam_a_input.transform(matrix)
    beam_b_input = beam_b_input.transform(matrix)
    output.append(((beam_a_input, beam_a), (beam_b_input, beam_b)))
  return output

def focii(data, beam_1, beam_2, plane):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]
  and return a list of the beam_1 focal points for beam_1 with beam_2 in the
  'T'angential or 'S'agittal plane. The beam_n are indexes 0 for beam_a or
  1 for beam_b.
  """
  return [(i[beam_1][0],
           Vector(focus(i[beam_1][1], j[beam_2][1], plane=plane)[0]))
           for i in data for j in data[data.index(i) + 1:]]

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

def draw_focii_vs_input_tangential(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the tangential focii against the beam-mirror input positions.
  """
  beam_0_focii = focii(data, 0, 0, 'T')
  beam_1_focii = focii(data, 1, 1, 'T')
  b0in, b0in_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,0,0].tolist()])
  b0x, b0x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,0].tolist()])
  b0y, b0y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,1].tolist()])
  b0z, b0z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,2].tolist()])
  b1in, b1in_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,0,0].tolist()])
  b1x, b1x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,0].tolist()])
  b1y, b1y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,1].tolist()])
  b1z, b1z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,2].tolist()])

  plt.figure(1, figsize=(18, 4), dpi=150)
  plt.suptitle("Tangential Focus Coordinates vs Beam Input Position",
      fontsize=STYLE['titlesize'])
  plt.subplot(1, 3, 1)
  plt.errorbar(b0in, b0x, xerr=b0in_err, yerr=b0x_err, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75, label="Tangential A-A")
  plt.errorbar(b1in, b1x, xerr=b1in_err, yerr=b1x_err, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75, label="Tangential B-B")
  plt.legend(loc=2, numpoints=1)
  plt.ylabel("Focus X coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 2)
  plt.errorbar(b0in, b0y, xerr=b0in_err, yerr=b0y_err, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75)
  plt.errorbar(b1in, b1y, xerr=b1in_err, yerr=b1y_err, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75)
  plt.ylabel("Focus Y coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 3)
  plt.errorbar(b0in, b0z, xerr=b0in_err, yerr=b0z_err, marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75)
  plt.errorbar(b1in, b1z, xerr=b1in_err, yerr=b1z_err, marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75)
  plt.ylabel("Focus Z coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.show()

def draw_focii_vs_input_sagittal(data):
  """Take a list of data in the form
      [((beam_a_input, beam_a), (beam_b_input, beam_b))]

  and draw the tangential focii against the beam-mirror input positions.
  """
  beam_0_focii = focii(data, 0, 1, 'S')
  beam_1_focii = focii(data, 1, 0, 'S')
  b0in, b0in_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,0,0].tolist()])
  b0x, b0x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,0].tolist()])
  b0y, b0y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,1].tolist()])
  b0z, b0z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,2].tolist()])
  b1in, b1in_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,0,0].tolist()])
  b1x, b1x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,0].tolist()])
  b1y, b1y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,1].tolist()])
  b1z, b1z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,2].tolist()])

  plt.figure(1, figsize=(18, 4), dpi=150)
  plt.suptitle("Sagittal Focus Coordinates vs Beam Input Position",
      fontsize=STYLE['titlesize'])
  plt.subplot(1, 3, 1)
  plt.errorbar(b0in, b0x, xerr=b0in_err, yerr=b0x_err, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75, label="Sagittal A-A")
  plt.errorbar(b1in, b1x, xerr=b1in_err, yerr=b1x_err, marker='o', ls='None',
      color=STYLE['bas_color'], alpha=0.75, label="Sagittal B-B")
  plt.legend(loc=2, numpoints=1)
  plt.ylabel("Focus X coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 2)
  plt.errorbar(b0in, b0y, xerr=b0in_err, yerr=b0y_err, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75)
  plt.errorbar(b1in, b1y, xerr=b1in_err, yerr=b1y_err, marker='o', ls='None',
      color=STYLE['bas_color'], alpha=0.75)
  plt.ylabel("Focus Y coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(1, 3, 3)
  plt.errorbar(b0in, b0z, xerr=b0in_err, yerr=b0z_err, marker='o', ls='None',
      color=STYLE['abs_color'], alpha=0.75)
  plt.errorbar(b1in, b1z, xerr=b1in_err, yerr=b1z_err, marker='o', ls='None',
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
  beam_0_focii = focii(data, 0, 0, 'T')
  beam_1_focii = focii(data, 1, 1, 'T')
  b0x, b0x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,0].tolist()])
  b0y, b0y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,1].tolist()])
  b0z, b0z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,2].tolist()])
  b1x, b1x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,0].tolist()])
  b1y, b1y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,1].tolist()])
  b1z, b1z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,2].tolist()])

  fig = plt.figure(1, figsize=(14, 9), dpi=150)
  zy_plot = plt.subplot(223)
  plt.errorbar(b0z, b0y, xerr=b0z_err, yerr=b0y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'], label='Tangential A-A')
  plt.errorbar(b1z, b1y, xerr=b1z_err, yerr=b1y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['bbt_color'], label='Tangential B-B')
  plt.legend(loc='best', numpoints=1)
  zx_plot = plt.subplot(221, sharex=zy_plot)
  plt.errorbar(b0z, b0x, xerr=b0z_err, yerr=b0x_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'])
  plt.errorbar(b1z, b1x, xerr=b1z_err, yerr=b1x_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['bbt_color'])
  xy_plot = plt.subplot(224, sharey=zy_plot)
  plt.errorbar(b0x, b0y, xerr=b0x_err, yerr=b0y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['aat_color'])
  plt.errorbar(b1x, b1y, xerr=b1x_err, yerr=b1y_err, marker='o', ls='None',
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
  beam_0_focii = focii(data, 0, 1, 'S')
  beam_1_focii = focii(data, 1, 0, 'S')
  b0x, b0x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,0].tolist()])
  b0y, b0y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,1].tolist()])
  b0z, b0z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_0_focii)[:,1,2].tolist()])
  b1x, b1x_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,0].tolist()])
  b1y, b1y_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,1].tolist()])
  b1z, b1z_err = zip(*[(i.value, i.maximal_error())
      for i in np.array(beam_1_focii)[:,1,2].tolist()])

  fig = plt.figure(2, figsize=(14, 9), dpi=150)
  zy_plot = plt.subplot(223)
  plt.errorbar(b0z, b0y, xerr=b0z_err, yerr=b0y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'], label='Sagittal A-B')
  plt.errorbar(b1z, b1y, xerr=b1z_err, yerr=b1y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['bas_color'], label='Sagittal B-A')
  plt.legend(loc='best', numpoints=1)
  zx_plot = plt.subplot(221, sharex=zy_plot)
  plt.errorbar(b0z, b0x, xerr=b0z_err, yerr=b0x_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'])
  plt.errorbar(b1z, b1x, xerr=b1z_err, yerr=b1x_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['bas_color'])
  xy_plot = plt.subplot(224, sharey=zy_plot)
  plt.errorbar(b0x, b0y, xerr=b0x_err, yerr=b0y_err, marker='o', ls='None',
      alpha=0.7, color=STYLE['abs_color'])
  plt.errorbar(b1x, b1y, xerr=b1x_err, yerr=b1y_err, marker='o', ls='None',
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
