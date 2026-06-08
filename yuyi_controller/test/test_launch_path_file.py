#!/usr/bin/env python3

import importlib.util
import os
from pathlib import Path
import unittest
import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
os.environ.setdefault("ROS_LOG_DIR", "/tmp/ros_logs")


def load_module(module_name: str, relative_path: str):
    module_path = REPO_ROOT / relative_path
    spec = importlib.util.spec_from_file_location(module_name, module_path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


class LaunchPathFileBehaviorTest(unittest.TestCase):
    def test_controller_launch_defaults_to_workspace_params_file(self):
        launch_module = load_module(
            "yuyi_controller_launch",
            "yuyi_controller/launch/yuyi_controller.launch.py",
        )

        self.assertEqual(
            launch_module.default_params_file(
                "/tmp/fake_ws/install/yuyi_controller/share/yuyi_controller"
            ),
            "/tmp/fake_ws/src/LidarBridge/yuyi_controller/config/yuyi_controller_params.yaml",
        )

    def test_controller_launch_uses_config_path_when_no_override_is_given(self):
        launch_module = load_module(
            "yuyi_controller_launch",
            "yuyi_controller/launch/yuyi_controller.launch.py",
        )

        parameters = launch_module.build_node_parameters("config.yaml", "")

        self.assertEqual(parameters, ["config.yaml"])

    def test_controller_launch_adds_override_when_path_file_is_explicit(self):
        launch_module = load_module(
            "yuyi_controller_launch",
            "yuyi_controller/launch/yuyi_controller.launch.py",
        )

        parameters = launch_module.build_node_parameters("config.yaml", "/tmp/generated_path.yaml")

        self.assertEqual(
            parameters,
            ["config.yaml", {"path_file": "/tmp/generated_path.yaml"}],
        )

    def test_simulation_launch_only_forwards_path_override_when_present(self):
        launch_module = load_module(
            "simulation_with_controller_launch",
            "simulation/launch/simulation_with_controller.launch.py",
        )

        no_override_arguments = launch_module.build_controller_launch_arguments("params.yaml", "")
        explicit_override_arguments = launch_module.build_controller_launch_arguments(
            "params.yaml",
            "/tmp/generated_path.yaml",
        )

        self.assertEqual(no_override_arguments, {"params_file": "params.yaml"})
        self.assertEqual(
            explicit_override_arguments,
            {
                "params_file": "params.yaml",
                "path_file": "/tmp/generated_path.yaml",
            },
        )

    def test_default_controller_config_points_to_workspace_generated_path(self):
        config_path = REPO_ROOT / "yuyi_controller/config/yuyi_controller_params.yaml"

        with config_path.open("r", encoding="utf-8") as stream:
            params = yaml.safe_load(stream)

        ros_params = params["yuyi_controller_node"]["ros__parameters"]
        self.assertEqual(
            ros_params["path_file"],
            "src/LidarBridge/simulation/paths/generated_path.yaml",
        )
        self.assertFalse(ros_params["loop_path"])


if __name__ == "__main__":
    unittest.main()
