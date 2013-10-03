"""
A package for manipulating itop mirror testing data and generating analysis
plots.
"""

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
from itop.math import Vector
import numpy as np
import math as sys_math
from itop.math.linalg import rotation_matrix
from itop.math.linalg import rotation_matrix_arrays
from itop.math.optics import focus
from itop.math.optics import refract
from itop.math.optics import fresnel_coefficients
from itop.math.optics import radius_from_normals as optical_radius
from itop.math.optics import reconstruct_mirror_normal as mirror_normal


def focii(data, alignment):
  """Return a nested dictionary containing each beam-pair crossing point
  in the data.

  The output dictionary has two keys: 'tangent' and 'sagittal' for the
  two crossing points. Each of those values is another dictionary with
  keys of the form 'nN' where n and N are the indexes of the beams used
  to compute the focii. The tangential dictionaries are lists of crossing
  points for beams with the same index. Likewise, the sagittal dictionaries
  are lists made up of only beams with differing indexes.

  Each element in a crossing point list is of the form
    '([beam_n_input, beam_N_input], [beam_n_focus, beam_N_focus])'
  """

  tan_focii = {}
  sag_focii = {}
  for i in range(len(data[0].beams)):
    for j in range(i, len(data[0].beams)):
      if j > i:
        sag_focii['{}{}'.format(i, j)] = []
      else:
        tan_focii['{}{}'.format(i, j)] = []

  for item_1, data_1 in enumerate(data):
    d1_inputs = alignment.input_positions([data_1.mirror_position, 0, 0])
    for item_2, data_2 in enumerate(data[item_1:]):
      d2_inputs = alignment.input_positions([data_2.mirror_position, 0, 0])
      item_2 = item_2 + item_1
      for beam_1_id, beam_1 in enumerate(data_1.beams):
        for beam_2_id, beam_2 in enumerate(data_2.beams):
          if beam_1_id == beam_2_id and item_2 > item_1:
            tan_focii['{}{}'.format(beam_1_id, beam_2_id)].append(
                ([d1_inputs[beam_1_id], d2_inputs[beam_2_id]],
                 focus(beam_1, beam_2, plane='T')))
          elif beam_2_id > beam_1_id:
            sag_focii['{}{}'.format(beam_1_id, beam_2_id)].append(
                ([d1_inputs[beam_1_id], d2_inputs[beam_2_id]],
                 focus(beam_1, beam_2, plane='S')))
  return {'tangent': tan_focii, 'sagittal': sag_focii}

def radius(
    output_beam_1, input_position_1,
    output_beam_2, input_position_2,
    input_beam_direction,
    mirror_index,
    lab_index = 1.000277,
    face_normal=None):
  """Return the radius of curvature from two output beam trajectories
  from a common input beam at two different input positions and the
  subtrate refraction index.

  Takes the optional arguments:
    lab_index (1.000277):
      The refraction index of the laboratory air.

    face_normal ([0,0,1]):
      The normal vector of the front face.
  """
  if output_beam_1 is output_beam_2:
    raise Exception('Radius Calculation requires two unique beams.')
  if face_normal is None:
    face_normal = [0, 0, 1]
  reflection_normal_1 = mirror_normal(
      output_beam_1.direction,
      input_beam_direction,
      face_normal, mirror_index, lab_index)
  reflection_normal_2 = mirror_normal(
      output_beam_2.direction,
      input_beam_direction,
      face_normal, mirror_index, lab_index)
  return optical_radius(
      reflection_normal_1,
      reflection_normal_2,
      input_beam_direction,
      input_position_1,
      input_position_2)

def radii(data, alignment, mirror_index, lab_index=1.000277):
  """Return a list of the radii between beam_1 and beam_2 in the given data
  as a tuple (s, r) where s is the absolute separation distance in x and r
  is the radius. Any additional keyword arguments are passed on to the
  mirror normal reconstruction.
  """
  matrix = rotation_matrix(alignment.beams[0].direction)
  input_directions = [-i.transform(matrix).direction for i in alignment.beams]
  beam_indexes = range(len(data[0].beams))
  mirror_radii = [[] for i in beam_indexes]
  for item_1, data_1 in enumerate(data[:-1]):
    input_1 = alignment.input_positions([data_1.mirror_position, 0, 0])
    for data_2 in data[item_1 + 1:]:
      input_2 = alignment.input_positions([data_2.mirror_position, 0, 0])
      for beam_id in beam_indexes:
        mirror_radii[beam_id].append(
            radius(data_1.beams[beam_id], input_1[beam_id],
                   data_2.beams[beam_id], input_2[beam_id],
                   input_directions[beam_id], mirror_index, lab_index))
  return mirror_radii

def _reflectance(
    power_in, power_out, theta_in, theta_out, index_in, index_out):
  trans_in = fresnel_coefficients(theta_in, index_out, index_in)['T']
  trans_out = fresnel_coefficients(theta_out, index_in, index_out)['T']
  return power_out / (power_in * trans_in * trans_out)


def reflectance(
    reflected_beam, original_beam, alignment,
    mirror_index, lab_index=1.000277, mirror_normal=None):
  """Return the reflectance of the mirror at the given input position
  given the reflected beam, the original beam, and the refractive
  indexes."""
  if mirror_normal is None:
    mirror_normal = [0, 0, 1]
  input_angle = sys_math.acos(
      (original_beam.transform(
          rotation_matrix(alignment.beams[0].direction)).direction
      ).dot(mirror_normal))
  exit_angle = sys_math.acos(
      -refract(
          -reflected_beam.direction, mirror_normal, lab_index, mirror_index
      ).dot(mirror_normal))
  transmittance_in = fresnel_coefficients(
      input_angle, lab_index, mirror_index)['T']
  transmittance_out = fresnel_coefficients(
      exit_angle, mirror_index, lab_index)['T']
  reflectivity = ((reflected_beam.power[0] / (
      original_beam.power[0] * transmittance_in * transmittance_out)
      ) * (original_beam.power[1] / reflected_beam.power[1]))
  return reflectivity

STYLE = {
    'aat_color': '#ee0000',
    'abt_color': '#ee6600',
    'bat_color': '#ee9900',
    'bbt_color': '#eecc00',
    'aas_color': '#0066ee',
    'abs_color': '#0000ee',
    'bas_color': '#00ccee',
    'bbs_color': '#0099ee',
    'labelsize': 14,
    'titlesize': 14}

BEAM_COLORS = {
    0: '#ee0000',
    1: '#ee6600',
    2: '#ee9900',
    4: '#eecc00',
    }

def draw_alignment(alignment):
  """Plot alignment diagnostics."""
  number_of_beams = alignment.calibration.data['number of beams']
  samples = {}
  errors = {}
  for index in range(number_of_beams):
    _samples, _errors = zip(
        *[(i.array(), i.errors()) for i in alignment.beams[index].samples])
    samples[index] = np.array(_samples) - np.mean(_samples, 0)
    errors[index] = abs(np.array(_errors))

  opts = {'marker':'o', 'ls':'None', 'alpha':0.5}
  fig, axes = plt.subplots(1, 2, figsize=(13, 4))
  for y_axis, x_axis in ((0, 2), (1, 2)):
    for beam in range(1, number_of_beams):
      axes[y_axis].errorbar(samples[beam][:, x_axis],
          ((samples[beam][:, y_axis]) + (samples[0][:, y_axis])),
          xerr=errors[beam][:, x_axis].T.tolist(),
          yerr=errors[beam][:, y_axis].T.tolist(),
          **opts)
      axes[y_axis].set_xlabel('{}-coordinate [mm]'.format('xyz'[x_axis]),
          fontsize=STYLE['labelsize'])
      axes[y_axis].set_ylabel(
          'delta {dim}{beam} - delta {dim}0 [mm]'.format(
              dim='xyz'[y_axis], beam=beam),
          fontsize=STYLE['labelsize'])

  plt.tight_layout()
  fig.suptitle('Alignment Diagnostics', y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("alignment.pdf")
  plt.show()

def draw_samples(data):
  """Draw the beam sample points in the frame of the tracker.
  Data must be a list of itop.DataPoints.
  """
  axes = plt.subplots(1, 3, figsize=(18, 4))[1]
  for data_point in data:
    for index, beam in enumerate(data_point.beams):
      if beam is not None:
        for plot, x_axis, y_axis in ((0, 2, 0), (1, 2, 1), (2, 0, 1)):
          samples = np.array([j.array() for j in beam.samples])
          axes[plot].plot(samples[:, x_axis], samples[:, y_axis],
              color=BEAM_COLORS[index], marker='o', alpha=0.5)
          axes[plot].set_xlabel('{}-coordinate [mm]'.format('xyz'[x_axis]),
              fontsize=STYLE['labelsize'])
          axes[plot].set_ylabel('{}-coordinate [mm]'.format('xyz'[y_axis]),
              fontsize=STYLE['labelsize'])

  plt.tight_layout()
  plt.suptitle("Beam Samples", y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("samples.pdf")
  plt.show()

def draw_radii(data, alignment, mirror_index, lab_index=1.000277):
  """Plot the radius of the mirror as a function of beam separation distance
  in x. The radius is computed from each of the beam pairs in the given data.
  Data points must be in the form of
      ((beam_a_input, beam_a), (beam_b_input, beam_b))).

  Any keyword arguments are passed on to the mirror normal reconstruction
  method.
  """
  n_beams = len(data[0].beams)
  axes = plt.subplots(1, n_beams, figsize=(6.0 * n_beams, 4))[1]
  for beam, r_list in enumerate(
        radii(data, alignment, mirror_index, lab_index)):
    axes[beam].set_title('Beam {}'.format(beam), fontsize=STYLE['titlesize'])
    axes[beam].plot(*zip(*r_list), marker='o', ls='None',
      color=BEAM_COLORS[beam], alpha=0.75)
    axes[beam].set_ylabel("Radius [mm]", fontsize=STYLE['labelsize'])
    axes[beam].set_xlabel("Beam Separation [mm]", fontsize=STYLE['labelsize'])

  plt.tight_layout()
  plt.suptitle("Calculated Radius vs Beam Separation", y=1.05,
      fontsize=STYLE['titlesize'])
  plt.savefig("radii.pdf")
  plt.show()

def draw_reflectance(data, alignment, mirror_index, lab_index=1.000277, mirror_normal=None):
  """Plot the reflectance of the mirror as a function of beam
  input position.
  """

  plt.figure(figsize=(9, 5), dpi=150)
  reflectivities = {}
  for index in range(len(alignment.beams)):
    reflectivities[index] = {'i':[], 'r':[], 'e':[]}
  for sample in data:
    input_positions = alignment.input_positions(
        [sample.mirror_position, 0, 0])
    for beam_index, reflected_beam in enumerate(sample.beams):
      r_value, r_error = reflectance(
          reflected_beam, alignment.beams[beam_index],
          alignment, mirror_index, lab_index, mirror_normal)
      reflectivities[beam_index]['i'].append(input_positions[beam_index][0])
      reflectivities[beam_index]['r'].append(r_value)
      reflectivities[beam_index]['e'].append(max(r_error))
  for index in reflectivities.keys():
    plt.errorbar(
        reflectivities[index]['i'],
        reflectivities[index]['r'],
        reflectivities[index]['e'],
        marker='o', ls='None', color=BEAM_COLORS[index],
        label='Beam {}'.format(index))
  plt.title('Reflectance', fontsize=STYLE['titlesize'])
  plt.legend(loc='best', numpoints=1)
  plt.ylabel("Reflectance", fontsize=STYLE['labelsize'])
  plt.xlabel("Input Position [mm]", fontsize=STYLE['labelsize'])
  plt.savefig("reflectance.pdf")
  plt.show()

def draw_focii(data, alignment):
  """Draw the beam crossing positions in the given set of mirror-aligned
  data."""
  cross_points = focii(data, alignment)
  for polarization in ('tangent', 'sagittal'):
    _draw_focii_input(cross_points[polarization], polarization)
    _draw_focii_space(cross_points[polarization], polarization)

def _draw_focii_input(focii_data, polarization):
  """Draw the focal points vs input position."""
  axes = plt.subplots(1, 3, figsize=(18, 4))[1]
  colors = [['red','orange'], ['blue','green']]
  for pair in sorted(focii_data.keys()):
    for sample in focii_data[pair]:
      for dimension in range(3):
        for point, beam in enumerate(pair):
          axes[dimension].plot(
              sample[0][0][0], sample[1][point][dimension],
              marker='o', alpha=0.5, color=colors[int(beam)][point])

  for dimension, axis in enumerate(axes):
    axis.set_xlabel('Input Position [mm]', fontsize=STYLE['labelsize'])
    axis.set_ylabel('{}-coordinate [mm]'.format('xyz'[dimension]),
        fontsize=STYLE['labelsize'])
  plt.tight_layout()
  plt.suptitle(
      "{} Focii vs Input Position".format(polarization.capitalize()),
      y=1.05, fontsize=STYLE['titlesize'])
  plt.savefig("focii_input_{}.pdf".format(polarization.lower()))
  plt.show()

def _draw_focii_space(focii_data, polarization):
  """Draw the focal points in 3D space."""
  fig = plt.figure(figsize=(14, 9))
  colors = [['red','orange'], ['blue','green']]
  axes = [plt.subplot(2, 2, 3)]
  axes.append(plt.subplot(2, 2, 1, sharex=axes[0]))
  axes.append(plt.subplot(2, 2, 4, sharey=axes[0]))
  for pair in sorted(focii_data.keys()):
    for sample in focii_data[pair]:
      for view, xy_axes in enumerate([(2, 1), (2, 0), (0, 1)]):
        for point, beam in enumerate(pair):
          axes[view].plot(
              sample[1][point][xy_axes[0]], sample[1][point][xy_axes[1]],
              marker='o', alpha=0.5, color=colors[int(beam)][point])

  for label in axes[1].axes.get_xticklabels():
    label.set_visible(False)
  for label in axes[2].axes.get_yticklabels():
    label.set_visible(False)

  axes[0].xaxis.set_major_locator(MaxNLocator(14, prune='both'))
  axes[0].yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  axes[1].xaxis.set_major_locator(MaxNLocator(9, prune='both'))
  axes[2].yaxis.set_major_locator(MaxNLocator(9, prune='both'))
  axes[0].set_xlabel('z coordinate [mm]', fontsize=STYLE['labelsize'])
  axes[0].set_ylabel('y coordinate [mm]', fontsize=STYLE['labelsize'])
  axes[1].set_ylabel('x coordinate [mm]', fontsize=STYLE['labelsize'])
  axes[2].set_xlabel('x coordinate [mm]', fontsize=STYLE['labelsize'])

  plt.tight_layout()
  fig.subplots_adjust(hspace=0, wspace=0)
  plt.suptitle('{} Focal Points'.format(polarization.capitalize()), y=1.05,
      fontsize=STYLE['titlesize'])
  plt.savefig('focii_space_{}.pdf'.format(polarization.lower()))
  plt.show()

