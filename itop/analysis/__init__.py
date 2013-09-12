"""
A package for manipulating itop mirror testing data and generating analysis
plots.
"""

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from itop.math import Vector
import numpy as np
from itop.math.linalg import rotation_matrix
from itop.math.linalg import rotation_matrix_arrays
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
  nominal_in_a = -mirror_center_from_calibration
  nominal_in_b = nominal_in_a + beam_separation

  beam_a_true_input = (nominal_in_a -
      nominal_in_a.dot(-alignment.beam_a.direction) *
      Vector(-alignment.beam_a.direction))
  beam_b_true_input = (nominal_in_b -
      nominal_in_b.dot(-alignment.beam_b.direction) *
      Vector(-alignment.beam_b.direction))
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
      if beam_1 == beam_2 and data.index(j) <= data.index(i):
        pass
      else:
        focal_point = focus(i[beam_1][1], j[beam_2][1], plane=plane)
        output.append(
            ((i[beam_1][0], focal_point[0]), (j[beam_2][0], focal_point[1])))
  return output

def radii(data, beam_1, beam_2, in_1, in_2, face_normal, index_in, index_out):
  """Return a list of the radii between beam_1 and beam_2 in the given data
  as a tuple (s, r) where s is the absolute separation distance in x and r
  is the radius. Any additional keyword arguments are passed on to the
  mirror normal reconstruction.
  """
  if beam_1 == beam_2:
    return [(i[beam_1][0].array()[0] - j[beam_2][0].array()[0],
             radius(mirror_normal(i[beam_1][1].direction.array(),
                        in_1, face_normal, index_in, index_out),
                    mirror_normal(j[beam_2][1].direction.array(),
                        in_2, face_normal, index_in, index_out),
                    in_1,
                    i[beam_1][0].array()[:2],
                    j[beam_2][0].array()[:2])
            ) for k, j in enumerate(data) for i in data[k:] if i is not j]
  else:
    return [(i[beam_1][0].array()[0] - j[beam_2][0].array()[0],
             radius(mirror_normal(i[beam_1][1].direction.array(),
                        in_1, face_normal, index_in, index_out),
                    mirror_normal(j[beam_2][1].direction.array(),
                        in_2, face_normal, index_in, index_out),
                    in_1,
                    i[beam_1][0].array()[:2],
                    j[beam_2][0].array()[:2])
            ) for j in data for i in data]



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

def draw_alignment(alignment):
  """Plot alignment diagnostics."""
  samples_a = np.array(
      [j.array() for j in alignment.beam_a.samples])
  samples_b = np.array(
      [j.array() for j in alignment.beam_b.samples])
  samples_a_err = np.array(
      [i.errors() for i in alignment.beam_a.samples]).T.tolist()
  samples_b_err = np.array(
      [i.errors() for i in alignment.beam_b.samples]).T.tolist()
  opts = {'marker':'o', 'ls':'None', 'alpha':0.5}

  fig, axes_a = plt.subplots(1, 3, figsize=(18, 4))
  axes_b = np.array([axis.twinx() for axis in axes_a])

  axes_a[0].errorbar(samples_a[:,2], samples_a[:,0],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='r', **opts)
  axes_b[0].errorbar(samples_b[:,2], samples_b[:,0],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='y', **opts)
  axes_a[0].set_xlabel('z-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_a[0].set_ylabel('Beam A x-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_b[0].set_ylabel('Beam B x-coordinate [mm]', fontsize=STYLE['labelsize'])

  axes_a[1].errorbar(samples_a[:,2], samples_a[:,1],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='r', **opts)
  axes_b[1].errorbar(samples_b[:,2], samples_b[:,1],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='y', **opts)
  axes_a[1].set_xlabel('z-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_a[1].set_ylabel('Beam A y-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_b[1].set_ylabel('Beam B y-coordinate [mm]', fontsize=STYLE['labelsize'])

  axes_a[2].errorbar(samples_a[:,0], samples_a[:,1],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='r', **opts)
  axes_b[2].errorbar(samples_b[:,0], samples_b[:,1],
      xerr=samples_a_err[1][2], yerr=samples_a_err[1][0], color='y', **opts)
  axes_a[2].set_xlabel('x-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_a[2].set_ylabel('Beam A x-coordinate [mm]', fontsize=STYLE['labelsize'])
  axes_b[2].set_ylabel('Beam B x-coordinate [mm]', fontsize=STYLE['labelsize'])

  plt.tight_layout()
  fig.suptitle('Alignment Diagnostics', y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("alignment.pdf")
  plt.show()

def draw_samples(data):
  """Draw the beam sample points in the frame of the tracker.
  Data points must be in the form of
      ((beam_a_input, beam_a), (beam_b_input, beam_b))).
  """
  plt.figure(figsize=(18, 4))
  for i in data:
      samples_a = np.array([j.array() for j in i[0][1].samples])
      samples_b = np.array([j.array() for j in i[1][1].samples])
      plt.subplot(131)
      plt.plot(samples_a[:,2], samples_a[:,0], color='r', marker='o', alpha=0.5)
      plt.plot(samples_b[:,2], samples_b[:,0], color='y', marker='o', alpha=0.5)
      plt.subplot(132)
      plt.plot(samples_a[:,2], samples_a[:,1], color='r', marker='o', alpha=0.5)
      plt.plot(samples_b[:,2], samples_b[:,1], color='y', marker='o', alpha=0.5)
      plt.subplot(133)
      plt.plot(samples_a[:,0], samples_a[:,1], color='r', marker='o', alpha=0.5)
      plt.plot(samples_b[:,0], samples_b[:,1], color='y', marker='o', alpha=0.5)
  plt.subplot(131)
  plt.ylabel("x-coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("z-coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(132)
  plt.ylabel("y-coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("z-coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(133)
  plt.ylabel("y-coordinate [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("x-coordinate [mm]", fontsize=STYLE['labelsize'])

  plt.tight_layout()
  plt.suptitle("Beam Samples", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("samples.pdf")
  plt.show()

def draw_radii(data, alignment, index_in=1.46, index_out=1.000277):
  """Plot the radius of the mirror as a function of beam separation distance
  in x. The radius is computed from each of the beam pairs in the given data.
  Data points must be in the form of
      ((beam_a_input, beam_a), (beam_b_input, beam_b))).

  Any keyword arguments are passed on to the mirror normal reconstruction
  method.
  """

  aa_radii = radii(data, 0, 0,
      -alignment.beam_a.direction.array(),
      -alignment.beam_a.direction.array(),
      [0,0,1],
      index_in, index_out)
  ab_radii = radii(data, 0, 1,
      -alignment.beam_a.direction.array(),
      -alignment.beam_b.direction.array(),
      [0,0,1],
      index_in, index_out)
  bb_radii = radii(data, 1, 1,
      -alignment.beam_b.direction.array(),
      -alignment.beam_b.direction.array(),
      [0,0,1],
      index_in, index_out)
  plt.figure(figsize=(10, 4), dpi=150)
  plt.subplot(121)
  plt.title('A-A Radii', fontsize=STYLE['titlesize'])
  plt.plot(*zip(*aa_radii), marker='o', ls='None',
      color=STYLE['aat_color'], alpha=0.75, label="A-A Radii")
  plt.ylabel("Radius [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Beam Separation [mm]", fontsize=STYLE['labelsize'])
  #plt.subplot(132)
  #plt.title('A-B Radii', fontsize=STYLE['titlesize'])
  #plt.plot(*zip(*ab_radii), marker='o', ls='None',
  #    color=STYLE['abt_color'], alpha=0.75, label="A-B Radii")
  #plt.ylabel("Radius [mm]", fontsize=STYLE['labelsize'])
  #plt.xlabel("Beam Separation [mm]", fontsize=STYLE['labelsize'])
  plt.subplot(122)
  plt.title('B-B Radii', fontsize=STYLE['titlesize'])
  plt.plot(*zip(*bb_radii), marker='o', ls='None',
      color=STYLE['bbt_color'], alpha=0.75, label="B-B Radii")
  plt.ylabel("Radius [mm]", fontsize=STYLE['labelsize'])
  plt.xlabel("Beam Separation [mm]", fontsize=STYLE['labelsize'])

  plt.tight_layout()
  plt.suptitle("Calculated Radius vs Beam Separation", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("radii.pdf")
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

  plt.figure(figsize=(18, 4), dpi=150)
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

  plt.tight_layout()
  plt.suptitle("Tangential Focus Coordinates vs Beam Input Position", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("focii-input_tangential.pdf")
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

  plt.figure(figsize=(18, 4), dpi=150)
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

  plt.tight_layout()
  plt.suptitle("Sagittal Focus Coordinates vs Beam Input Position", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("focii-input_sagittal.pdf")
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

  fig = plt.figure(figsize=(14, 9), dpi=150)
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

  plt.tight_layout()
  fig.subplots_adjust(hspace=0, wspace=0)
  plt.suptitle("Tangential Focal Points in Mirror Coordinate System", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("focii-space_tangential.pdf")
  plt.show()

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

  fig = plt.figure(figsize=(14, 9), dpi=150)
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

  plt.tight_layout()
  fig.subplots_adjust(hspace=0, wspace=0)
  plt.suptitle("Sagittal Focal Points in Mirror Coordinate System", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("focii-space_sagittal.pdf")
  plt.show()
