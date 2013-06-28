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

from itop.beam.focus import FocalPoint
from itop.beam.profiler import Profiler
from itop.beam.profiler import Tracker
from itop.motioncontrol.controller import StageController
import os

_ROOT = os.path.abspath(os.path.dirname(__file__))
def data_path(path):
      path = os.path.join(_ROOT, 'data', path)
      if os.path.exists(path):
        return path
      else:
        return None

