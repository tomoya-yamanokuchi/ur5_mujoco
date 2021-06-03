import numpy


import numpy as np


class ConfigSettingDclawSimulationEnvironment:
  def __init__(self):
      self.seed                 = 42
      self.xml_path             = "./model/ur5.xml"
      self.is_viewer_off_screen = False
      self.sequence             = 1
      self.step                 = 50

      self.camera_position      = {"x_coordinate" : 0,
                                   "y_coordinate" : 0,
                                   "z_coordinate" : 0.15,
                                   "elevation"    : -45,
                                   "azimuth"      : 180,
                                   "distance_rate": 0.8}

      self.claw_jnt_range_lb    = np.deg2rad([-90,-90,-90,   -90,-90,-90,   -90,-90,-90])
      self.claw_jnt_range_ub    = np.deg2rad([ 90, 90, 90,    90, 90, 90,    90, 90, 90])

      self.valve_jnt_range_lb   = -4*np.pi
      self.valve_jnt_range_ub   =  4*np.pi
