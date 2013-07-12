"""
Miscellanous shorthand and utility functions to simplify iTOP measurements.
"""

def clamp(value, min_value, max_value):
  """Constrain a value to between a minimum and maximum.

  """
  return max(min(max_value, value), min_value)
