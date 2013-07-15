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

from itop.math import Value, Vector
from itop.motioncontrol import StageController
from itop.beam import Beam, Profiler, Tracker, DataPoint, Alignment, Instrument

import os

_ROOT = os.path.abspath(os.path.dirname(__file__))
def data_path(path):
  """Returns the path of the requested local data path.

  """
  return os.path.join(_ROOT, 'data', path)

