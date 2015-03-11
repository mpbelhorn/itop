"""
iTOP: A python based interface to iTOP Mirror Testing
=====================================================

Documentation is available in the docstrings.

Contents
--------
This package provides several subpackages.

Subpackages
-----------
Using any of these subpackages requires an explicit import.

::

 beam                      --- Beam tracking
 math                      --- Optics, linear algebra
 motioncontrol             --- Interface to EPS30x Stages
 utilities                 --- Misc. funtions.
"""

from itop import math

N_HPFS = math.optics.index_quartz()
N_AIR = 1.000277

from itop import utilities
from itop import motioncontrol
from itop import beam
from itop import prism
from itop import photodiode
from itop import analysis

from itop.photodiode import Photodiode
from itop.math import Value, Vector
from itop.motioncontrol import StageController
from itop.beam import (
    Beam, Profiler, Tracker, DataPoint, Alignment, Instrument, Calibration)

import os

_ROOT = os.path.abspath(os.path.dirname(__file__))
def data_path(path, create=False):
  """Returns the path of the requested local data path.

  """
  if os.path.exists('/lab/data'):
    target = os.path.join('/lab/data', path)
  else:
    target = os.path.join(_ROOT, 'data', path)
  if create and not os.path.exists(os.path.dirname(target)):
    os.makedirs(os.path.dirname(target))
  return target



#####################################################################
INSTRUMENTS = {
    'source monitor': '/dev/ttyUSB0',
    'transmit monitor': '/dev/ttyUSB1',
    'esp 300': '/dev/ttyUSB2',
    'esp 301': '/dev/ttyUSB3',
    'profiler': '/dev/ttyUSB4',
    }

def instrumentation(extended=False):
  """Return instances of the raw instrumentation interfaces."""
  instruments = {
      'profiler':Profiler(INSTRUMENTS['profiler']),
      'esp 0':StageController(INSTRUMENTS['esp 300'],
                limits=[250, [-45.0, 190.0], 125.0]),
      'esp 1':StageController(INSTRUMENTS['esp 301'],
               limits=[125.0, [-0.1, 50.0], [-95.0, 125.0]]),
      'photodiode 0':Photodiode(INSTRUMENTS['source monitor']),
      'photodiode 1':Photodiode(INSTRUMENTS['transmit monitor']),
      }
  instruments['tracker'] = Tracker(
      instruments['esp 1'],
      instruments['esp 0'].axes[1],
      instruments['profiler'],
      instruments['photodiode 0'],
      xyz_axes=[1,2,3],
      reference_azimuth=180)
  instruments['mirror'] = instruments['esp 0'].axes[0]
  keys = ['profiler', 'esp 0', 'esp 1', 'photodiode 0', 'photodiode 1']
  if extended:
    keys += ['tracker','mirror']
  return tuple((instruments[k] for k in keys))


def initialize_instruments():
  """A rough initializer for the profiler and stage controllers.

  This function returns instances of the instrument equipment dependencies
  with hardcoded options, especially the device names as they appear on
  M. Belhorn's development machine. This function SHOULD NOT BE USED
  for production code. It is intended SOLELY FOR DEBUGGING PURPOSES!
  """
  profiler = Profiler('/dev/ttyUSB4')
  esp_300 = StageController('/dev/ttyUSB2',
      limits=[250.0, [-45.0, 190.0], 125.0])
  esp_301 = StageController('/dev/ttyUSB3',
      limits=[125.0, [-0.05, 50.0], [-95.0, 125.0]])
  beam_monitor = Photodiode('/dev/ttyUSB0')
  rotator = esp_300.axes[1]
  tracker = Tracker(esp_301, rotator, profiler, beam_monitor)
  # mirror = esp_300.axes[0]
  # instrument = Instrument(
  #    tracker, mirror, os.path.join(_ROOT, 'data/alignment/test.gz'), 0.0)
  return (
      profiler,
      esp_300,
      esp_301,
      tracker,
      # instrument
      )


