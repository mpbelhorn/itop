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

 beam                      --- Beam alignment and tracking
 math                      --- Optics, linear algebra
 motioncontrol             --- Interface to EPS30x Stages
 utilities                 --- Misc. funtions.
"""

from beam.alignment import BeamAlignment
from beam.focus import FocalPoint
from beam.profiler import Profiler
from motioncontrol.controller import StageController
