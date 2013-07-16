# -*- coding: utf-8 -*-
"""A module to manage mathematics associated with metrology and position
uncertainty.
"""

import numpy as np
from numpy.linalg import norm
from numpy import array, sqrt, sin, cos

class MeasurementException(Exception):
  """Exceptions thrown by itop Measurement module objects."""
  def __init__(self, message):
    """Constructor for MeasurementException. Takes a message."""
    super(MeasurementException, self).__init__(message)
    self.message = message

  def __string__(self):
    return repr(self.message)


class Value(object):
  """A representation of a measured value with associated error.

  The basic arithmetic operators +,-,*, and / are supported assuming the value
  is a scalar. Operators work with other Value objects and normal scalar types,
  which are treated as error-free values.

  """
  def __init__(self, value, error=None):
    """Constructor for Point. Takes either an existing Value object or a scalar
    value.

    Optionally, a measurement error on the value can be passed at construction.
    If no error is given, the error on the value is set to 0 to represent a
    mathematical value instead of a measured value.

    Measurement errors can be passed a number of ways.
      1.) A single value is taken as an absolute error for each dimension.
          i.e. uncertainties=±Δx
      2.) An iterable +/- pair is taken as +/- uncertainties for each
          dimension.
          i.e. uncertainties=(-x, +x)

    In any case, values are stored internally as a set of +/- error pairs
    such that any input sign case is sorted or taken as absolute as needed.

    If a pair of +/- errors are non-zero and have the same sign, an exception
    is thrown.

    """
    try:
      self.value = float(value.value)
      self.error = tuple((float(err) for err in value.error))
    except AttributeError:
      self.value = float(value)
      if error is None:
        self.error = (-0.0, 0.0)
      else:
        try:
          if len(error) == 2 and error[0] * error[1] <= 0.0:
            self.error = tuple(sorted(error))
          else:
            raise MeasurementException(
                "Bad iterable input to measured Value error.")
        except TypeError:
          self.error = (-abs(error), abs(error))

  def __repr__(self):
    if self.error[0] == -self.error[1]:
      return '{0: >-.4g} ± {1[1]: <-.3g}'.format(
          self.value, self.error)
    else:
      return '{0: >-.4g} ({1[1]: >+.3g}/{1[0]: <+.3g})'.format(
          self.value, self.error)

  def __str__(self):
    """str(self) must return only the value so it can be used as input to
    itop.motioncontrol movement commands.

    """
    return str(self.value)

  def __float__(self):
    return float(self.value)

  def __iter__(self):
    for part in [self.value, self.error]:
      yield part


  def __add__(self, other):
    other = Value(other)
    return Value(self.value + other.value,
        tuple(float((j * 2 - 1) *
            sqrt(i[0]**2 + i[1]**2))
            for j, i in enumerate(zip(self.error, other.error))))

  def __sub__(self, other):
    other = Value(other)
    return Value(self.value - other.value,
        tuple(float((j * 2 - 1) * sign(self.value, other.value) *
            sqrt(i[0]**2 + i[1]**2))
            for j, i in enumerate(zip(self.error, other.error))))

  def __mul__(self, other):
    other = Value(other)
    return Value(self.value * other.value,
        tuple(float((j * 2.0 - 1.0) * sign(self.value, other.value) * sqrt(
            (other.value * i[0])**2 +
            (self.value * i[1])**2))
            for j, i in enumerate(zip(self.error, other.error))))

  def __rmul__(self, other):
    return self.__mul__(other)

  def __rdiv__(self, other):
    return Value(other) / self

  def __div__(self, other):
    other = Value(other)
    return Value(self.value / other.value,
        tuple(float((j * 2 - 1) * sign(self.value, other.value) * sqrt(
            (i[0] / other.value)**2 +
            (i[1] * self.value / (other.value**2))**2))
            for j, i in enumerate(zip(self.error, other.error))))

  def __pow__(self, other):
    other = Value(other)
    return Value(self.value ** other.value,
        tuple(float((j * 2 - 1) * sqrt(
          (other.value * (self.value**(other.value - 1.0)) * i[0])**2 +
          ((self.value**other.value) * np.log(self.value) * i[1])**2))
          for j, i in enumerate(zip(self.error, other.error))))

  def __abs__(self):
    return (
        abs(self.value), tuple(
        sorted(
            [sign(self.value, self.value) * self.error[0],
             sign(self.value, self.value) * self.error[1]]
        )))

  def __lt__(self, other):
    other = Value(other)
    if (self.value + self.error[1] < other.value + other.error[0]):
      return True
    else:
      return False

  def __le__(self, other):
    other = Value(other)
    if (self.value + self.error[0] <= other.value + other.error[1]):
      return True
    else:
      return False

  def __eq__(self, other):
    other = Value(other)
    if ((self.value + self.error[0] <= other.value + other.error[1]) and
        (self.value + self.error[1] >= other.value + other.error[0])):
      return True
    else:
      return False

  def __ne__(self, other):
    return not self.__eq__(other)

  def __gt__(self, other):
    other = Value(other)
    if (self.value + self.error[0] > other.value + other.error[1]):
      return True
    else:
      return False

  def __ge__(self, other):
    other = Value(other)
    if (self.value + self.error[1] >= other.value + other.error[0]):
      return True
    else:
      return False

  def maximal_error(self):
    """Returns the maximal absolute error."""
    return max((abs(i) for i in self.error))

  def minimal_error(self):
    """Returns the minimal absolute error."""
    return min((abs(i) for i in self.error))

def sign(first, second):
  """Returns -1 if the signs of a and b are different
  and 1 if they are the same.

  """
  return -1.0 if first * second < 0 else 1.0

class Vector(object):
  """A representation of a measured N-dimensional vector and its associated
  uncertainty.

  """
  def __init__(self, values, errors=None):
    """Constructor for Vector. Takes either an existing Vector object or
    an iterable of coordinates for each dimension.

    Optionally, measurement errors for each dimension can be passed at
    construction. If no measurement errors are given, the error for each
    dimension is set to 0, representing a mathematical vector as opposed
    to a metrological vector.

    Measurement errors can be passed a number of ways.
      1.) A single value is taken as an absolute error for each dimension.
          i.e. uncertainties=±Δx
      2.) An iterable +/- pair is taken as +/- uncertainties for each
          dimension.
          i.e. uncertainties=(-x, +x)
      3.) A list of an N-dimensional iterable of values of the form given in
          (1) and (2) sets the ith dimension error as per (1) or (2).
          i.e. uncertainties=[[Δx1, (-x2, +x2), Δx3, ..., (-xN, +xN)]]
    In any case, values are stored internally as a set of +/- error pairs
    such that any input sign case is sorted or taken as absolute as needed.

    If a pair of +/- errors are non-zero and have the same sign, an exception
    is thrown.

    """
    if isinstance(values, Vector):
      # It's a Vector obect already. Clone it.
      self.values = array(values.values)
      self.dimensions = len(self.values)
    else:
      try:
        if len(values) > 1:
          if all((isinstance(i, Value) for i in values)):
            # It's a list of Value objects. We're done.
            self.values = array(values)
            self.dimensions = len(self.values)
          else:
            if errors is None:
              # Default mathematical vector.
              self.values = array([Value(i, 0.0) for i in values])
              self.dimensions = len(self.values)
            else:
              try:
                if len(errors) == 1 and len(errors[0]) == len(values):
                  # Per dimension errors.
                  self.values = array(
                      [Value(i,j) for i,j in zip(values, errors[0])])
                  self.dimensions = len(self.values)
                elif len(errors) == 2:
                  # Global asymmetric errors.
                  self.values = array([Value(i, errors) for i in values])
                  self.dimensions = len(self.values)
                else:
                  print errors
                  raise MeasurementException(
                      """Error input cannot be interperted.""")
              except TypeError:
                # Not an iterable. Must be global absolute error.
                self.values = array([Value(i, errors) for i in values])
                self.dimensions = len(self.values)
        else:
          raise MeasurementException(
              """Vector requires more than one component.""")
      except TypeError:
        raise MeasurementException(
            """Vector expects a list of values.""")

  def __iter__(self):
    for value in self.values:
      yield value

  def __getitem__(self, key):
    try:
      return Vector(self.values[key])
    except MeasurementException:
      return Value(self.values[key])

  def __len__(self):
    return len(self.values)

  def __delitem__(self, item):
    raise TypeError("""Vector class doesn't support item deletion.
        Make a projection into a new Vector instead.""")

  def __setitem__(self, key, value):
    self.values[key] = Value(value)

  def __lt__(self, other):
    try:
      other = Vector(other)
      return all(pair[0] < pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def __le__(self, other):
    try:
      other = Vector(other)
      return all(pair[0] <= pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def __eq__(self, other):
    try:
      other = Vector(other)
      return all(pair[0] == pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def __ne__(self, other):
    try:
      other = Vector(other)
      return any(pair[0] != pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def __ge__(self, other):
    try:
      other = Vector(other)
      return all(pair[0] <= pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def __gt__(self, other):
    try:
      other = Vector(other)
      return all(pair[0] > pair[1] for pair in zip(self.values, other.values))
    except MeasurementException:
      return NotImplemented

  def project(self, indexes):
    """Projects the vector onto a sub- or super-space.

    Takes a list of indexes corresponding to the current dimensions of the
    vector in the order and dimensionality desired for the projected vector.
    Casting to a superspace requires 'None' in the relevant higher dimensions.

    Examples:
    * To project a vector of the form [x, y, z] onto the zy plane
      subspace: indexes=[2, 1]

    * To project a vector of the form [x,z] into the xyz superspace:
      indexes=[0, None, 1]
    """
    return Vector([(self[i] if i is not None else Value(0)) for i in indexes])

  def __add__(self, other):
    other = Vector(other)
    return Vector(self.values + other.values)

  def __sub__(self, other):
    other = Vector(other)
    return Vector(self.values - other.values)

  def __mul__(self, scalar):
    return Vector(self.values * scalar)

  def __rmul__(self, other):
    return self.__mul__(other)

  def __div__(self, other):
    return self.__mul__(1.0 / other)

  def __repr__(self):
    return repr([i for i in self])

  def __abs__(self):
    return (norm([i.value for i in self.values]),
        (-norm(self.lower_errors()), norm(self.upper_errors())))

  def array(self):
    """Returns the vector as a numpy array without errors."""
    return array([i.value for i in self.values])

  def errors(self):
    """Returns a list of the vector error tuples."""
    return [i.error for i in self.values]

  def upper_errors(self):
    """Returns a list of the positive errors."""
    return [i.error[0] for i in self.values]

  def lower_errors(self):
    """Returns a list of the negative errors."""
    return [i.error[1] for i in self.values]

  def maximal_errors(self):
    """Returns a list of the greatest absolute error in each dimension."""
    return [max((abs(limit) for limit in pair)) for pair in self.errors()]

  def minimal_errors(self):
    """Returns a list of the minimum absolute error in each dimension."""
    return [min((abs(limit) for limit in pair)) for pair in self.errors()]

  def normalize(self):
    """Returns a normalized copy of the vector with normalized errors."""
    return Vector(self / abs(self)[0])

  def dot(self, other):
    """Returns the dot product of this vector with another vector object."""
    return Vector([i[0] * i[1] for i in zip(self, Vector(other))])

  def rotate(self, axis, theta):
    """Returns the Vector rotated about the given axis by the given angle.

    Input axis can be given as any iterable 3D vector, including other Vector
    objects. Input angle can be either a scalar or a Vector object. If a
    Vector object is passed for either input, the associated error is
    propagated.

    """
    try:
      axis = axis.normalize()
    except AttributeError:
      axis = Vector(axis, 0.0).normalize()

    try:
      half_theta = theta.value / 2
      delta_theta = max([abs(i) for i in theta.error])
    except AttributeError:
      half_theta = theta / 2
      delta_theta = 0

    if axis.dimensions != self.dimensions:
      raise MeasurementException(
        """Vector and rotation axis of differing dimensions!
        Use consistant dimensionality to ensure meaningful output.""")
    elif (self.dimensions < 2) or (3 < self.dimensions):
      raise MeasurementException(
        """Euler-Rodriguez rotation cannot be used for 4(+)D vectors
        or single points.""")
    elif self.dimensions == 2:
      # Pad the vectors to 3D in order to compute the rotation.
      raise MeasurementException("2D rotations not implemeneted yet.")

    # Euler-Rodriguez Parameters (erps)
    erps = [cos(half_theta)]
    for i in axis * sin(half_theta):
      erps.append(i.value)
    delta_erps = [
        sin(half_theta) / 2 * delta_theta,
        sqrt(
            (axis[0].value * cos(half_theta) / 2 * delta_theta)**2 +
                (sin(half_theta) * axis.maximal_errors()[0])**2),
        sqrt(
            (axis[1].value * cos(half_theta) / 2 * delta_theta)**2 +
                (sin(half_theta) * axis.maximal_errors()[1])**2),
        sqrt(
            (axis[2].value * cos(half_theta) / 2 * delta_theta)**2 +
                (sin(half_theta) * axis.maximal_errors()[2])**2)]

    matrix = array(
        [[erps[0]*erps[0]+erps[1]*erps[1]-erps[2]*erps[2]-erps[3]*erps[3],
          2*(erps[1]*erps[2]-erps[0]*erps[3]),
          2*(erps[1]*erps[3]+erps[0]*erps[2])],
         [2*(erps[1]*erps[2]+erps[0]*erps[3]),
          erps[0]*erps[0]+erps[2]*erps[2]-erps[1]*erps[1]-erps[3]*erps[3],
          2*(erps[2]*erps[3]-erps[0]*erps[1])],
         [2*(erps[1]*erps[3]-erps[0]*erps[2]),
          2*(erps[2]*erps[3]+erps[0]*erps[1]),
          erps[0]*erps[0]+erps[3]*erps[3]-erps[1]*erps[1]-erps[2]*erps[2]]])

    delta_matrix = array([[0, 0, 0],
                          [0, 0, 0],
                          [0, 0, 0]])
    delta_matrix[0][0] = delta_matrix[1][1] = delta_matrix[2][2] = sqrt(
        (2*erps[0]*delta_erps[0])**2 + (2*erps[1]*delta_erps[1])**2 +
        (2*erps[2]*delta_erps[2])**2 + (2*erps[3]*delta_erps[3])**2)
    delta_matrix[0][1] = delta_matrix[1][0] = sqrt(
        (2*erps[3]*delta_erps[0])**2 + (2*erps[2]*delta_erps[1])**2 +
        (2*erps[1]*delta_erps[2])**2 + (2*erps[0]*delta_erps[3])**2)
    delta_matrix[0][2] = delta_matrix[2][0] = sqrt(
        (2*erps[2]*delta_erps[0])**2 + (2*erps[3]*delta_erps[1])**2 +
        (2*erps[0]*delta_erps[2])**2 + (2*erps[1]*delta_erps[3])**2)
    delta_matrix[1][2] = delta_matrix[2][1] = sqrt(
        (2*erps[1]*delta_erps[0])**2 + (2*erps[0]*delta_erps[1])**2 +
        (2*erps[3]*delta_erps[2])**2 + (2*erps[2]*delta_erps[3])**2)

    return Vector(np.dot(matrix, self.array()),
        [[sum([sqrt(
            (matrix[i][j] * self.maximal_errors()[i])**2 +
            (self[i].value * delta_matrix[i][j])**2
            ) for i in [0, 1, 2]]) for j in [0, 1, 2]
        ]])

