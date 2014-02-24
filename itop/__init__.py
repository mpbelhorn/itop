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
def data_path(path):
  """Returns the path of the requested local data path.

  """
  return os.path.join(_ROOT, 'data', path)


#####################################################################
INSTRUMENTS = {
    'source monitor': '/dev/ttyUSB0',
    'transmit monitor': '/dev/ttyUSB1',
    'esp 300': '/dev/ttyUSB2',
    'esp 301': '/dev/ttyUSB3',
    'profiler': '/dev/ttyUSB4',
    }

def instrumentation():
  """Return instances of the raw instrumentation interfaces."""
  return (
      Profiler(INSTRUMENTS['profiler']),
      StageController(INSTRUMENTS['esp 300'],
          limits=[250, [-45.0, 190.0], 125.0]),
      StageController(INSTRUMENTS['esp 301'],
          limits=[125.0, [-0.1, 50.0], [-95.0, 125.0]]),
      Photodiode(INSTRUMENTS['source monitor']),
      Photodiode(INSTRUMENTS['transmit monitor']),
      )

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
      limits=[125.0, [0.0, 50.0], [-95.0, 125.0]])
  beam_monitor = Photodiode('/dev/ttyUSB0')
  rotator = esp_300.axes[1]
  tracker = Tracker(esp_301, rotator, profiler, beam_monitor)
  mirror = esp_300.axes[0]
  #instrument = Instrument(
  #    tracker, mirror, os.path.join(_ROOT, 'data/alignment/test.gz'), 0.0)
  return (
      profiler,
      esp_300,
      esp_301,
      tracker,
      # instrument
      )


