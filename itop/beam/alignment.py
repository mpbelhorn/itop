# -*- coding: utf-8 -*-
"""
A class to manage the alignment of beams to the tracker.

"""
import ConfigParser
import ast
import os

from itop.beam import Beam
from itop.math import Vector
from itop.utilities import clamp
import datetime



class Alignment(object):
  """A class to establish the alignment between a tracker and the beams.

  """

  INTERFERENCE_CUTOFF = 40.0 # mm off the optical axis.

  def __init__(self, calibration):
    """Constructor for alignment."""
    self.beam_a = None
    self.beam_b = None
    self.displacement = None  # r_b(x,y,0) - r_a(x,y,0) in tracker frame.
    self.calibration = calibration
    self.date = None

  def align(self, tracker, home=False):
    """Establishes the alignment between the given tracker and the beams.

    Takes an optional keyword argument
    home (False): Recalibrates the rotation stage to it's home switch.

    """
    print 'Measuring tracker alignment. This will take some time.'
    self.beam_a = Beam(-1)
    self.beam_b = Beam(-1)
    tracker.devices['r_stage'].power_on()
    if home:
      tracker.devices['r_stage'].go_to_home(wait=True)
    tracker.devices['r_stage'].position(180, wait=True)
    tracker.facing_z_direction = 1
    shutter = tracker.devices['driver'].shutter_state
    shutter(0, 1)
    shutter(1, 0)
    self.beam_a = tracker.find_beam_trajectory(
        [tracker.axes[0].limits.upper,
         tracker.axes[1].limits.lower + 11,
         tracker.axes[2].limits.upper],
        -1, -1, z_samples=25)
    shutter(0, 0)
    shutter(1, 1)
    self.beam_b = tracker.find_beam_trajectory(
        self.beam_a.last_sample() + [-30, -5, 0],
        -1, 1, z_samples=25)
    self.displacement = self.beam_b.intercept - self.beam_a.intercept
    self.date = datetime.datetime.now().isoformat()

    # Rotate camera to face mirror.
    tracker.devices['r_stage'].position(0, wait=True)
    tracker.facing_z_direction = -1
    shutter(0, 1)

  def alignment_date(self):
    """Returns the date and time the current alignment data was taken.

    """
    if self.date is None:
      return 'invalid'
    else:
      return self.date

  def base_input_positions(self):
    """Return a list of the input positions for each beam when the mirror is at
    position (0,0,0).
    """
    inputs = [self.calibration.primary_input()]
    for beam, separation in zip((self.beam_b,), (self.displacement,)):
      nominal_input = inputs[0] + separation
      inputs.append(
          nominal_input - (
            nominal_input.dot(-beam.direction) * (-beam.direction)))
    return inputs

  def mirror_positions(self, input_position):
    """Return a list of the mirror positions for each beam to be at the given
    input position.
    """
    return [i - Vector(input_position) for i in self.base_input_positions()]

  def mirror_limits(self, input_range):
    """
    Return the mirror stage position limits that fully accomodate the
    given input x-position range.
    """
    limit_candidates = []
    for scan_limit in input_range:
      beam_pair = []
      for beam in self.mirror_positions([scan_limit, 0, 0]):
        beam_pair.append(clamp(beam[0].value, -250, 250))
      limit_candidates.append(beam_pair)
    if input_range[1] > input_range[0]:
      return (max(limit_candidates[0]), min(limit_candidates[1]))
    else:
      return (min(limit_candidates[0]), max(limit_candidates[1]))

  def input_positions(self, mirror_position):
    """Return a list of the beam input positions given the mirror position."""
    return [i - Vector(mirror_position) for i in self.base_input_positions()]

  def out_of_range(self, beam_index, mirror_position):
    """Return True if the beam with given index cannot be sampled due to any
    of the following conditions:
        profiler interference
        beam misses mirror
    """
    input_x = self.input_positions(mirror_position)[beam_index][0].value
    if (abs(input_x) < Alignment.INTERFERENCE_CUTOFF):
      return True
    elif (input_x < Calibration.EXTENTS['mirror'][0][0]) or (
          Calibration.EXTENTS['mirror'][0][1] < input_x):
      return True
    else:
      return False


class Calibration(object):
  """Data required to calibrate the instrumentation.

  The data is divided into three sections. Calibration displacements that do
  not change between fixed mirror and beam orientations are treated as
  constants. Calibration displacements that do change between mirrors or
  beam positions are treated as variables. Lastly, the approximate physical
  extents of the optical elements are stored as constants. The extents data
  is not intended to be accurate, but exists only to consolidate the
  nominal dimensions in a single place for loose logic decisions made
  in other modules which require that information.

  See the note M.Belhorn "iTOP Mirror Testing" under the section
  Mirror-Tracker Calibration for more details.

  Default values for each data section are stored in class property
  dictionaries, and are on the order of actual values.

  Instance copies of the data share the same keys as the defaults. Instance
  values are loaded from an INI-style configuration file at startup. The
  displacement data must be in the form of an itop.Vector and appears in the
  configuration files in the format

      "key = [x, y, z]|[(-ex,+ex), (-ey,+ey), (-ez,+ez)]"

  where 'key' is the name of the displacement; 'x','y','z' are the coordinates
  of the vector; and 'ex', 'ey', and 'ez' are the measurement uncertainties.

  """
  CONSTANTS = {
      'calibration_mirror':   Vector([-120.000, 0.000,    0.000], 0.0005),
      'calibration_tracker':  Vector([-125.000, 0.000,   95.000], 0.0005),
      'registration_mirror':  Vector([ 225.000, 0.000,    0.000], 0.001),
      'registration_tracker': Vector([ -32.004, 0.000,    6.600], 0.01),
      'reference_tracker':    Vector([  -4.026, 0.000,  -10.250], 0.01),
      'reference_delta':      Vector([   0.000, 0.000, 1447.000], 0.8),
      }

  VARIABLES = {
      'reference_mirror':Vector([33.220, 0.000, 10.832], 0.01),
      }

  EXTENTS = {
    'mirror':[(-225, 225), (-20, 0), (-100, 0)]
    }

  def __init__(self, configuration=None):
    """
    Constructs a calibration object.

    """
    self._path = configuration
    self.data = {}
    if configuration is not None:
      self.load(configuration)
    else:
      self.data.update(Calibration.CONSTANTS)
      self.data.update(Calibration.VARIABLES)

  def __repr__(self):
    return 'Calibration({})'.format(
        'default' if self._path is None else os.path.basename(self._path))

  def displacement(self):
    """Return the vector displacement of the tracker origin from the mirror
    optical axis at the mirror stage system origin."""
    return sum((i for k, i in self.data.iteritems() if k in self.data))

  def primary_input(self):
    """Return the input position of the primary beam when the mirror stage
    is at position (0, 0).
    """
    return self.data['calibration_mirror'] + self.data['registration_mirror']

  def save(self, path):
    """Save the calibration data to the given path."""
    config = ConfigParser.RawConfigParser()
    config.add_section('Constants')

    config.add_section('Variables')
    for key in Calibration.VARIABLES:
      _save_vector(config, 'Variables', key, self.data[key])

    for key in Calibration.CONSTANTS:
      _save_vector(config, 'Constants', key, self.data[key])

    with open(path, 'wb') as configfile:
      config.write(configfile)
    self._path = path

  def load(self, path):
    """Load calibration data from the given path."""
    config = ConfigParser.RawConfigParser()
    config.read(path)
    self._path = path

    for key in Calibration.VARIABLES:
      self.data[key] = _load_vector(config, 'Variables', key)

    for key in Calibration.CONSTANTS:
      self.data[key] = _load_vector(config, 'Constants', key)


def _load_vector(config, section, key):
  """Return a vector from a configuration file."""
  data = [ast.literal_eval(i) for i in config.get(section, key).split('|')]
  return Vector(data[0], [data[1]])

def _save_vector(config, section, key, item):
  """Return a vector from a configuration file."""
  entry = '{}|{}'.format(item.array().tolist(), item.errors())
  config.set(section, key, entry)

