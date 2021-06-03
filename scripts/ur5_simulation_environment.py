import copy
import numpy as np
import mujoco_py
from mujoco_py.modder   import TextureModder, CameraModder
from transforms3d.euler import euler2quat, quat2euler
# from model_geometry_names_handler import ModelGeometryNamesHandler



class UR5SimulationEnvironment:
    def __init__(self, config):
        self.seed                 = config.seed
        self.xml_path             = config.xml_path
        self.model                = mujoco_py.load_model_from_path(config.xml_path)
        self.sim                  = mujoco_py.MjSim(self.model)
        self.texture_modder       = TextureModder(self.sim)
        self.camera_modder        = CameraModder(self.sim)
        self.is_viewer_off_screen = config.is_viewer_off_screen
        self.viewer               = self.select_viewer()
        self.camera_position      = config.camera_position
        self.claw_jnt_range_lb    = config.claw_jnt_range_lb
        self.claw_jnt_range_ub    = config.claw_jnt_range_ub
        self.valve_jnt_range_lb   = config.valve_jnt_range_lb
        self.valve_jnt_range_ub   = config.valve_jnt_range_ub
        # self._valve_jnt_id        = self.sim.model.joint_name2id('valve_OBJRx')
        # self._target_bid          = self.model.body_name2id('target')
        # self._target_sid          = self.model.site_name2id('tmark')
        # self.geom_names_hander    = ModelGeometryNamesHandler()
        self.set_random_seed()
        # self.set_geom_rgb()
        # self.set_texture_rgb()
        # self.set_camera_position()
        return None


    def select_viewer(self):
        if self.is_viewer_off_screen:
            return mujoco_py.MjRenderContextOffscreen(self.sim, 0)
        else:
            return mujoco_py.MjViewer(self.sim)


    def set_random_seed(self):
        np.random.seed(self.seed)


    # def set_geom_rgb(self):
    #     # my_red   = [230,  74,  25]
    #     my_red   = [209,  49,  45]
    #     my_white = [255, 255, 255]
    #     my_green = [ 27, 176,  27]
    #     my_black = [ 38 , 38 , 38]

    #     self.rgb = dict()
    #     self.set_geom_names_randomize_target()
    #     self.rgb[self.__geom_names_randomize_target[0]] = np.array(my_green)
    #     self.rgb[self.__geom_names_randomize_target[1]] = np.array(my_white)
    #     for i in range(len(self.__geom_names_randomize_target))[2:-3]:
    #         self.rgb[self.__geom_names_randomize_target[i]] = np.array(my_black)
    #     for i in range(len(self.__geom_names_randomize_target))[-3:]:
    #         self.rgb[self.__geom_names_randomize_target[i]] = np.array(my_red)


    # def set_geom_names_randomize_target(self):
    #     self.__geom_names_randomize_target = self.geom_names_hander.get_all_names()


    # def set_texture_rgb(self):
    #     for name in self.__geom_names_randomize_target:
    #         self.texture_modder.set_rgb(name, self.rgb[name])


    def render(self):
        self.viewer.render()


    def check_camera_pos(self):
        self.sim.reset()
        for i in range(100):
            diff = 0.1
            self.model.cam_pos[0][0] = self.rs.uniform(self.default_cam_pos[0][0] - diff, self.default_cam_pos[0][0] + diff)  # x-axis
            self.model.cam_pos[0][1] = self.rs.uniform(self.default_cam_pos[0][1] - diff, self.default_cam_pos[0][1] + diff)  # x-axis
            self.model.cam_pos[0][2] = self.rs.uniform(self.default_cam_pos[0][2] - diff, self.default_cam_pos[0][2] + diff)  # z-axis
            self.sim.step()
            self.render()


    def set_camera_position(self):
        self.viewer.cam.lookat[0] = self.camera_position["x_coordinate"]
        self.viewer.cam.lookat[1] = self.camera_position["y_coordinate"]
        self.viewer.cam.lookat[2] = self.camera_position["z_coordinate"]
        self.viewer.cam.elevation = self.camera_position["elevation"]   # camera rotation around the axis in the plane going through the frame origin (if 0 you just see a line)
        self.viewer.cam.azimuth   = self.camera_position["azimuth"]     # camera rotation around the camera's vertical axis
        self.viewer.cam.distance  = self.model.stat.extent * self.camera_position["distance_rate"]


    def get_state(self, is_only_qpos_qpos):
        env_state = copy.deepcopy(self.sim.get_state())
        if is_only_qpos_qpos:
            env_state = {"qpos": env_state.qpos, "qvel": env_state.qvel}
        return env_state


    def set_state(self, qpos, qvel):
        if qpos is None: qpos = np.zeros(self.model.nq)
        if qvel is None: qvel = np.zeros(self.model.nv)
        if not (qpos.shape == (self.model.nq,) and qvel.shape == (self.model.nv,)):
            print("\n***************************************")
            print(" model.nq = {} | qpos.nq = {}".format(self.model.nq, qpos.shape[0]))
            print(" model.nv = {} | qvel.nv = {}".format(self.model.nv, qvel.shape[0]))
            print("\n   dimension is incorrect ")
            print("\n***************************************")
            raise AssertionError("^^^^^^^^^")
        old_state = self.sim.get_state()
        new_state = mujoco_py.MjSimState(old_state.time, qpos, qvel,
                                         old_state.act, old_state.udd_state)

        self.sim.set_state(new_state)
        self.sim.data.ctrl[:9] = qpos[:9]
        self.sim.data.ctrl[9:] = 0.0
        self.sim.forward()



    def reset_env(self, qpos_init=None, qvel_init=None, target_position_init=None):
        self.sim.reset()
        # self.set_jnt_range()
        # self.set_target_position(target_position_init)
        # self.set_state(qpos=qpos_init, qvel=qvel_init)
        self.render()


    def set_target_position(self, target_position):
        self._target_position = target_position


    # def set_jnt_range(self):
    #     # --- claw ---
    #     for jnt_index in range(9):
    #         self.sim.model.jnt_range[jnt_index, 0] = self.claw_jnt_range_lb[jnt_index]
    #         self.sim.model.jnt_range[jnt_index, 1] = self.claw_jnt_range_ub[jnt_index]

    #     # --- valve ---
    #     self.sim.model.jnt_range[self._valve_jnt_id, 0] = self.valve_jnt_range_lb
    #     self.sim.model.jnt_range[self._valve_jnt_id, 1] = self.valve_jnt_range_ub


    def step(self):
        self.sim.step()


    def set_control_input(self, ctrl):
        assert ctrl.shape[0] == 9
        self.sim.data.ctrl[:9] = ctrl