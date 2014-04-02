"""
A package for handeling prism testing.
"""

import os
import matplotlib.pyplot as plt
from math import sin, cos, acos, atan, degrees, sqrt
from itop.math.measurements import ratio_error, power_error
from itop.math.optics import fresnel_coefficients, snells_law
import numpy as np
from itop import N_HPFS, N_AIR


def deflection_angle(undeflected_beam, deflected_beam):
  """Return the angle of beam deflection in radians."""
  return acos(undeflected_beam.direction.dot(deflected_beam.direction).value)

def prism_angle(alpha, index_quartz=N_HPFS, index_air=N_AIR):
  """Return the prism angle given the beam's angle of deflection
  when the prism is moved into the beam path.

  """
  return atan(sin(alpha) / ((index_quartz / index_air) - cos(alpha)))

def angle_from_beams(undeflected_beam, deflected_beam, **kwargs):
  """Return the prism angle in degrees from a pair of deflected and
  undeflected beam trajectories.

  Any keyword arguments are passed to the deflection angle calculation
  itop.prism_angle().
  """
  return degrees(prism_angle(
      deflection_angle(undeflected_beam, deflected_beam), **kwargs))

def transmission_raw(data):
  """Return the background subtracted raw transmission coefficient from
  a set of data in the form

      [signal, signal_background, reference, reference_background]

  where each element is a 2D list. The first sublist column is the beam
  monitor photodiode reading and the second is the downstream photodiode
  reading."""
  # averages = [np.mean(i, axis=0) for i in data]
  # uncertainties = [np.std(i, axis=0) / np.sqrt(len(i)) for i in data]
  backgrounds = [np.mean(i, axis=0) for i in (data[1], data[3])]
  intensities = np.array(
          [((data[s][:, 1] - backgrounds[s/2][1]) /
            (data[s][:, 0] - backgrounds[s/2][0])) for s in (0, 2)])
  signal, reference = np.mean(intensities, axis=1)
  signal_error, reference_error = (
          np.std(intensities, axis=1) / np.sqrt(len(intensities[0])))
  return (signal / reference,
          ratio_error(signal, reference, signal_error, reference_error))

def _fresnel_correction(theta_in, index_lab, index_optic, **kwargs):
  """Return the Fresnel loss correction factor T_in * T_out for
  a beam passing through a plate optic with refractive index index_optic
  from an environment with index index_lab at an incidence angle theta_in
  radians."""
  theta_in_error = kwargs.pop('theta_error', 0.003)
  fresnel_in = fresnel_coefficients(
          theta_in,
          index_lab, index_optic)['T']
  fresnel_out = fresnel_coefficients(
          snells_law(index_lab, index_optic, theta_in),
          index_optic, index_lab)['T']
  fresnel_correction = fresnel_in * fresnel_out
  fresnel_correction_error = fresnel_correction - (
          fresnel_coefficients(
              theta_in + theta_in_error,
              index_lab, index_optic)['T'] *
          fresnel_coefficients(
              snells_law(index_lab, index_optic, theta_in + theta_in_error),
              index_lab, index_optic)['T'])
  return (fresnel_correction, fresnel_correction_error)

def transmission_corrected(
      data,
      theta_in=0.008,
      index_optic=N_HPFS,
      index_lab=N_AIR):
  """Return the background subtracted and Fresnel loss corrected internal
  transmission coefficient from a set of data in the form

    [signal, signal_background, reference, reference_background]

  where each element is a 2D list. The first sublist column is the beam
  monitor photodiode reading and the second is the downstream photodiode
  reading. The Fresnel correction requires the input angle theta_in
  (in radians) and the indexes of refraction."""
  t_correction, t_correction_error = _fresnel_correction(
          theta_in, index_lab, index_optic)
  t_raw, t_raw_error = transmission_raw(data)
  return (t_raw / t_correction,
          ratio_error(t_raw, t_correction, t_raw_error, t_correction_error))

def transmission_per_unit_length(data, **kwargs):
  """Return the background subtracted and Fresnel loss corrected internal
  transmission per unit length from a set of data in the form

      [signal, signal_background, reference, reference_background]

  where each element is a 2D list. The first sublist column is the beam
  monitor photodiode reading and the second is the downstream photodiode
  reading.

  Keyword arguments:
      'length' (0.450): Beam path length in desired units through optic.
      'length_error (0.001): Uncertainty on length.

  All other keywords are passed to 'transmission_corrected'.
  """
  length = kwargs.pop('length', 0.450)
  length_error = kwargs.pop('length_error', 0.00001)
  t_corrected, t_corrected_error = transmission_corrected(data, **kwargs)
  scale = 1.0 / float(length)
  scale_error = ratio_error(1.0, float(length), 0, length_error)
  t_error = power_error(t_corrected, scale, t_corrected_error, scale_error)
  return (t_corrected**scale, t_error)

def transmission(data, **kwargs):
  """Return the background subtracted and Fresnel loss corrected internal
  transmission per unit length from a set of data with statistical error
  and an estimate of the systematic error. Data must be in the form

        [signal, signal_background, reference, reference_background]

  where each element is a 2D list. The first sublist column is the beam
  monitor photodiode reading and the second is the downstream photodiode
  reading.

  Keyword arguments:
      'length' (0.450): Beam path length in desired units through optic.
      'length_error' (0.00001): Uncertainty on length.
      'theta_in' (0.008): Incident angle (radians) to the optic.
      'index_lab' (itop.N_AIR): Lab refractive index.
      'index_optic' (itop.N_HPFS): Optic refractive index.
  """
  length = kwargs.pop('length', 0.450)
  length_error = kwargs.pop('length_error', 0.00001)
  theta_in = kwargs.pop('theta_in', 0.007)
  index_optic = kwargs.pop('index_optic', N_HPFS)
  index_lab = kwargs.pop('index_lab', N_AIR)
  systematics = [
      np.std([transmission_per_unit_length(
              data, length=length, length_error=length_error,
              theta_in=theta_in + i,
              index_optic=index_optic,
              index_lab=index_lab)[0] for i in (-0.007, 0.007)]),
      np.std([transmission_per_unit_length(
              data, length=length, length_error=length_error,
              theta_in=theta_in,
              index_optic=index_optic + i,
              index_lab=index_lab)[0] for i in (-0.0003, 0.0003)]),
      np.std([transmission_per_unit_length(
              data, length=length, length_error=length_error,
              theta_in=theta_in,
              index_optic=index_optic,
              index_lab=index_lab + i)[0] for i in (-0.0003, 0.0003)]),
  ]
  output = list(transmission_per_unit_length(
          data, length=length, length_error=length_error,
          theta_in=theta_in,
          index_optic=index_optic,
          index_lab=index_lab))
  output.append(sum(systematics))
  return tuple(output)

def plot_transmittance(data_sets, **kwargs):
  prism_id = kwargs.pop('prism_id', '')
  force_overwrite = kwargs.pop('overwrite', False)

  results = np.array([transmission(d, **kwargs) for d in data_sets])

  fig, axis = plt.subplots(1, 1, figsize=(10,5))
  number_of_trials = len(results)
  ordinates = range(1, number_of_trials + 1)
  plt.errorbar(
      ordinates,
      [i[0]*100. for i in results],
      yerr=[(i[1]+i[2])*100. for i in results],
      ls='None', marker='o')
  axis.yaxis.get_major_formatter().set_useOffset(False)
  plt.xlim(0, number_of_trials + 1)
  plt.ylim(99.5, 100)

  avg = np.mean(results[:,0]) * 100
  avg_error = (
      (sqrt(sum([i**2 for i in [j[1] + j[2] for j in results]])) * 100) /
       sqrt(len(results)))
  plt.axhline(avg, color='b', ls='--')
  axis.text(
      0.95, 0.03,
      'Avg = {:03.3f}$\pm${:03.3f}'.format(avg, avg_error),
      verticalalignment='bottom', horizontalalignment='right',
      transform=axis.transAxes, color='b', fontsize=15)
  plt.title('Prism ' + prism_id + ' Transmittance', fontsize=18)
  plt.ylabel('T [%/m]', fontsize=18)
  tick_labels = [str(i) for i in ordinates]
  tick_labels.insert(0, '')
  plt.xticks(range(number_of_trials + 1), tick_labels);
  plt.xlabel('Trial', fontsize=18)
  axis.yaxis.set_tick_params(labelsize=12)
  axis.xaxis.set_tick_params(labelsize=12)
  output_path = '/lab/data/prisms/{}/transmittance_{}.pdf'.format(
      prism_id, prism_id)
  if not os.path.exists(output_path) or force_overwrite:
    print('Saving output')
    plt.savefig(output_path)
  else:
    print('Output not saved!')
  plt.show()
