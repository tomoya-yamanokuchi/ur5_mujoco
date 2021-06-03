import numpy as np
from ur5_simulation_environment import UR5SimulationEnvironment
from config_setting_dclaw_simulation_environment import ConfigSettingDclawSimulationEnvironment
from joint_space_target_example import JointSpaceTargetExample


config               = ConfigSettingDclawSimulationEnvironment()
env                  = UR5SimulationEnvironment(config)
joint_target_example = JointSpaceTargetExample()


sequence   = 1
step       = 5
inner_step = 300

ctrl = np.stack([
        joint_target_example.target1,
        joint_target_example.target2,
        joint_target_example.target3,
        joint_target_example.target4,
        joint_target_example.target5
])


env.reset_env()
for index_step in range(step):
    # env.set_control_input(ctrl[index_step])
    for index_inner_step in range(inner_step):
        env.render()
        env.step()


