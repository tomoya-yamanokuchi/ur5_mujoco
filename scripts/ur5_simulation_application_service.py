import numpy as np
from ur5_simulation_environment import UR5SimulationEnvironment
from config import Config


class UR5SimulationApplicationService:
    def run(self):
        config = Config()
        env    = UR5SimulationEnvironment(config)
        step   = 300

        env.reset_env()
        env.set_difference_control_input(np.array([0.3]*6))
        for index_step in range(step):
            env.render()
            env.step()


if __name__ == '__main__':
    sim = UR5SimulationApplicationService()
    sim.run()
