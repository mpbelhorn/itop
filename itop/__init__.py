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
from itop import analysis

from itop.math import Value, Vector
from itop.motioncontrol import StageController
from itop.beam import Beam, Profiler, Tracker, DataPoint, Alignment, Instrument

import os

_ROOT = os.path.abspath(os.path.dirname(__file__))
def data_path(path):
  """Returns the path of the requested local data path.

  """
  return os.path.join(_ROOT, 'data', path)


#####################################################################

def initialize_instruments():
  """A rough initializer for the profiler and stage controllers.

  This function returns instances of the instrument equipment dependencies
  with hardcoded options, especially the device names as they appear on
  M. Belhorn's development machine. This function SHOULD NOT BE USED
  for production code. It is intended SOLELY FOR DEBUGGING PURPOSES!
  """
  profiler = Profiler('/dev/ttyUSB0')
  esp_300 = StageController('/dev/ttyUSB1',
      limits=[250.0, [-45.0, 190.0], 125.0])
  esp_301 = StageController('/dev/ttyUSB2',
      limits=[125.0, [0.0, 25.0], [-95.0, 125.0]])
  return (profiler, esp_300, esp_301)

