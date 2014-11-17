"""
Functions for analyzing mirror surface distortion.
"""

import matplotlib.pyplot as plt
from numpy import array
from math import sqrt

from matplotlib import rcParams
rcParams.update({'figure.autolayout': True})

def _power_bands(data):
  pls = (data[:,7]/max(data[:,7])).tolist()
  return [i for i, j in enumerate(pls) if j > 0.5]


def draw_power(data, serial, x_position='-105'):
  inputs = -data[:,0]
  pls = (data[:,7]/max(data[:,7])).tolist()
  power_band_indexes = _power_bands(data)
  plt.plot(inputs, pls, marker='.')
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.title("SN{} Self-Normalized Reflected Beam Power".format(serial))
  plt.ylabel("Power / Max(Power)")
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.tight_layout()
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_power.eps'.format(
        serial, x_position))

def draw_height(data, serial, x_position='-105'):
  inputs = -data[:,0]
  power_band_indexes = _power_bands(data)
  plt.plot(-data[:,0], data[:,1], label="13.5%", marker='.')
  plt.plot(-data[:,0], data[:,2], label="50.0%", marker='.')
  plt.plot(-data[:,0], data[:,3], label="80.0%", marker='.')
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.legend()
  plt.title("SN{} Reflection Height".format(serial))
  plt.ylabel("Profile Height [mm]")
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.tight_layout()
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_height.eps'.format(
        serial, x_position))


def draw_width(data, serial, x_position='-105'):
  inputs = -data[:,0]
  power_band_indexes = _power_bands(data)
  plt.plot(-data[:,0], data[:,4], label="13.5%", marker='.')
  plt.plot(-data[:,0], data[:,5], label="50.0%", marker='.')
  plt.plot(-data[:,0], data[:,6], label="80.0%", marker='.')
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.legend()
  plt.title("SN{} Reflection Width".format(serial))
  plt.ylabel("Profile Width [mm]")
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.tight_layout()
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_width.eps'.format(
        serial, x_position))


def draw_distortion(data, serial, x_position='-105'):
  power_band_indexes = _power_bands(data)
  inputs = -data[:,0]
  plt.plot(inputs, data[:,1]/data[:,4], label="13.5%", marker='.')
  plt.plot(inputs, data[:,2]/data[:,5], label="50.0%", marker='.')
  plt.plot(inputs, data[:,3]/data[:,6], label="80.0%", marker='.')
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.axhline(1, color='black', ls='--')
  lgd = plt.legend(loc='center left', bbox_to_anchor=(1, 0.5))
  plt.title("SN{} Distortion".format(serial))
  plt.ylabel("Profile Height/Width")
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.tight_layout()
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_classic.eps'.format(
        serial, x_position),
      bbox_extra_artists=(lgd,), bbox_inches='tight')

def eccentricity(dt, index=0):
  return array([sqrt(1 - (i/j)**2) if i<j else sqrt(1 - (j/i)**2) for i,j in zip(dt[:,1+index],dt[:,4+index])])

def eccentricity_err(dt, index=0):
  return array(
    [sqrt(((b * 40.) / (a**2 * sqrt(1 - (b/a)**2)))**2 + ((b**2 * 40.) / (a**3 * sqrt(1 - (b/a)**2)))**2)
      if b < a else sqrt(
        ((a * 40.) / (b**2 * sqrt(1 - (a/b)**2)))**2 + ((a**2 * 40.) / (b**3 * sqrt(1 - (a/b)**2)))**2)
    for a, b in zip(dt[:,1+index],dt[:,4+index])])

def draw_eccentricity(data, serial, x_position='-105'):
  power_band_indexes = _power_bands(data)
  inputs = -data[:,0]
  plt.errorbar(inputs, eccentricity(data, 0), eccentricity_err(data, 0),
      0.03, ls='-', marker='.', label="13.5%")
  plt.errorbar(inputs, eccentricity(data, 1), eccentricity_err(data, 1),
      0.03, ls='-', marker='.', label="50.0%")
  plt.errorbar(inputs, eccentricity(data, 2), eccentricity_err(data, 2),
      0.03, ls='-', marker='.', label="80.0%")
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.axhline(0, color='black', ls='--')
  plt.legend(loc=4)
  plt.title("SN{} Profile Eccentricity".format(serial))
  plt.ylabel('Eccentricity e = \sqrt{1-(\frac{b}{a})^2}')
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_eccentricity.eps'.format(
        serial, x_position))

def draw_e2(data, serial, x_position='-105'):
  power_band_indexes = _power_bands(data)
  inputs = -data[:,0]
  plt.plot(inputs, eccentricity(data, 0)/np.sqrt(1-eccentricity(data, 0)**2),
      ls='-', marker='.', label="13.5%")
  plt.plot(inputs, eccentricity(data, 1)/np.sqrt(1-eccentricity(data, 1)**2),
      ls='-', marker='.', label="50.0%")
  plt.plot(inputs, eccentricity(data, 2)/np.sqrt(1-eccentricity(data, 2)**2),
      ls='-', marker='.', label="80.0%")
  plt.xticks(arange(len(data)/4)+1)
  plt.axvline(inputs[power_band_indexes[0]], color='red', ls='--')
  plt.axvline(inputs[power_band_indexes[-1]], color='red', ls='--')
  plt.axhline(0, color='black', ls='--')
  plt.legend(loc=4)
  plt.title("SN{} Second Degree Profile Eccentricity".format(serial))
  plt.ylabel('e" = e/sqrt(1-e^2)')
  plt.xlabel("Input Y Position (from S1) [mm]")
  plt.savefig(
      '/lab/data/mirrors/{}/distortion_x{}_eccentricity2.eps'.format(
        serial, x_position))
