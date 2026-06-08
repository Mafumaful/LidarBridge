#!/usr/bin/env python3

import importlib.util
import os
from pathlib import Path
import unittest


REPO_ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")


def load_module(module_name: str, relative_path: str):
    module_path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class SimulationLaunchDefaultsTest(unittest.TestCase):
    def test_ackermann_launch_defaults_to_workspace_params_file(self):
        launch_module = load_module(
            "ackermann_simulator_launch",
            "simulation/launch/ackermann_simulator.launch.py",
        )

        self.assertEqual(
            launch_module.default_params_file("/tmp/fake_ws/install/simulation/share/simulation"),
            "/tmp/fake_ws/src/LidarBridge/simulation/config/ackermann_simulator_params.yaml",
        )

    def test_path_generator_defaults_to_workspace_config_file(self):
        launch_module = load_module(
            "path_generator_launch",
            "simulation/launch/path_generator.launch.py",
        )
        script_module = load_module(
            "path_generator_script",
            "simulation/scripts/path_generator.py",
        )

        expected = "/tmp/fake_ws/src/LidarBridge/simulation/config/path_generator_params.yaml"
        self.assertEqual(
            launch_module.default_config_file("/tmp/fake_ws/install/simulation/share/simulation"),
            expected,
        )
        self.assertEqual(
            str(script_module.default_config_file(Path("/tmp/fake_ws/install/simulation/share/simulation"))),
            expected,
        )

    def test_simulation_with_map_defaults_to_workspace_map(self):
        launch_module = load_module(
            "simulation_with_map_launch",
            "simulation/launch/simulation_with_map.launch.py",
        )

        self.assertEqual(
            launch_module.default_map_yaml_file("/tmp/fake_ws/install/simulation/share/simulation"),
            "/tmp/fake_ws/src/LidarBridge/simulation/maps/simulation_map.yaml",
        )

    def test_simulation_with_controller_defaults_to_workspace_controller_config(self):
        launch_module = load_module(
            "simulation_with_controller_launch",
            "simulation/launch/simulation_with_controller.launch.py",
        )

        self.assertEqual(
            launch_module.default_controller_params_file(
                "/tmp/fake_ws/install/yuyi_controller/share/yuyi_controller"
            ),
            "/tmp/fake_ws/src/LidarBridge/yuyi_controller/config/yuyi_controller_params.yaml",
        )


if __name__ == "__main__":
    unittest.main()
