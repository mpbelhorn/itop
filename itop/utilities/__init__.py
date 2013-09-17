"""
Miscellanous shorthand and utility functions to simplify iTOP measurements.
"""
import zlib
import cPickle

def clamp(value, min_value, max_value):
  """Constrain a value to between a minimum and maximum.

  """
  return max(min(max_value, value), min_value)

def save_object(target, file_path):
  """Saves an object to a gzipped serialized object file.

  """
  with open(file_path, 'wb') as output_file:
    output_file.write(zlib.compress(
        cPickle.dumps(target, cPickle.HIGHEST_PROTOCOL),9))

def load_object(file_path):
  """Loads an object from a gzipped serialized object file created
  with itop.utilities.save_object()

  """
  try:
    with open(file_path, 'rb') as input_file:
      try:
        pickled_data = zlib.decompress(input_file.read())
        return cPickle.loads(pickled_data)
      except AttributeError as oops:
        # Ooops. Changed the implementation of something.
        print "Failed to load object from", file_path, '(' + str(oops) + ')'
        return None
  except IOError:
    print "Failed to read file."
    return None

