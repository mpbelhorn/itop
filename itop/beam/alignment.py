from beam import Beam
import numpy as np
import json

class BeamAlignment(object):
  """
  For establishing alignment and positioning of LBP with respect to beams.
  """
  def __init__(self, controller, group_id, camera):
    """
    Set pointers to the LBP and stage group controller.
    """
    self.controller = controller
    self.group_id = group_id
    self.camera = camera
    self.beam_a = Beam(self.controller, self.group_id, self.camera)
    self.beam_b = Beam(self.controller, self.group_id, self.camera)
    self.height_offset = None
    self.angles = [None, None, None]
    self.x_displacement = None
    self.y_displacement = None

  def findTrajectories(self):
    """
    Initilizes the trajectories of both upstream beams.
    """
    # Block beam 'B' and find beam 'A' trajectory.
    raw_input("Clear beam 'A' and block beam 'B'. Press enter to continue.")
    self.beam_a.findTrajectory()
    # Block beam 'A' and find beam 'B' trajectory.
    self.height_offset = None
    while self.height_offset is None:
      try:
        response = raw_input(
            "Clear beam 'B' and block beam 'A'.\n"
            " Make sure camera is at proper height. "
            "Enter the vertical offset (Enter for None): ")
        self.height_offset = 0.0 if response is '' else float(response)
      except ValueError:
        print "Offset must be a signed floating-point number or nothing."
    self.beam_b.findTrajectory()
    self.x_displacement, self.y_displacement = (
        self.beam_b.r_initial - self.beam_a.r_initial +
        np.array([0, self.height_offset, 0]))[:2]
    self.angles = [angle for angle in self.beam_a.angles()]

    def save(self, file_path):
      """
      Saves the beam alignment data to a JSON.
      """
      data = {}
      data['height_offset'] = self.height_offset
      data['angles'] = self.angles
      data['x_displacement'] = self.x_displacement
      data['y_displacement'] = self.y_displacement

      with open(file_path, 'wb') as fp:
          json.dump(data, fp)

    def load(self, file_path):
      """
      Loads the beam alignment data from a saved file.
      """
      with open(file_path, 'rb') as fp:
          data = json.load(fp)
      self.height_offset = data['height_offset']
      self.angles = data['angles']
      self.x_displacement = data['x_displacement']
      self.y_displacement = data['y_displacement']
