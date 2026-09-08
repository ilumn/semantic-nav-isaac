import os
import tempfile
import unittest
from pathlib import Path

from run_semantic_nav import simulation_app_launch_config
from semantic_nav_isaac_scene.runtime_args import (
    build_parser,
    expand_template,
    infer_ros_package_mapping,
    plan_xacro_expansion,
    prepare_importable_urdf,
    stage_mode,
    urdf_requires_xacro,
    validate_stage_path,
)


class RuntimeArgumentTests(unittest.TestCase):
    def test_no_arguments_is_valid_interactive_launch(self):
        args = build_parser().parse_args([])
        self.assertFalse(args.headless)
        self.assertIsNone(args.stage_path)

    def test_rtx_launch_forces_single_gpu(self):
        config = simulation_app_launch_config(headless=False)
        self.assertIs(config["multi_gpu"], False)
        self.assertIs(config["headless"], False)

    def test_environment_template_requires_every_variable(self):
        self.assertEqual(expand_template("/opt/ros/${ROS_DISTRO}", {"ROS_DISTRO": "jazzy"}), "/opt/ros/jazzy")
        self.assertIsNone(expand_template("/opt/ros/${ROS_DISTRO}", {}))

    def test_stage_modes_and_suffix_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "scene.usd"
            self.assertEqual(stage_mode(None), "memory")
            self.assertEqual(stage_mode(path), "compose")
            path.write_text("#usda 1.0\n", encoding="utf-8")
            self.assertEqual(stage_mode(path), "open")
            self.assertEqual(stage_mode(path, rebuild=True), "compose")
            self.assertEqual(validate_stage_path(path), path.resolve())
            with self.assertRaises(ValueError):
                validate_stage_path(Path(directory) / "scene.txt")

    def test_package_mapping_uses_original_ros_package(self):
        with tempfile.TemporaryDirectory() as directory:
            package = Path(directory) / "turtlebot3_description"
            (package / "urdf").mkdir(parents=True)
            (package / "meshes").mkdir()
            source = package / "urdf" / "robot.urdf"
            source.write_text("<robot name='r'/>", encoding="utf-8")
            self.assertEqual(
                infer_ros_package_mapping(source, "turtlebot3_description"),
                {"name": "turtlebot3_description", "path": str(package.resolve())},
            )

    def test_xacro_plan_detects_nominal_urdf_and_empty_namespace(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "robot.urdf"
            source.write_text(
                '<robot xmlns:xacro="http://www.ros.org/wiki/xacro"><xacro:arg name="namespace"/>'
                '<link name="${namespace}base_link"/></robot>',
                encoding="utf-8",
            )
            executable = root / "xacro"
            executable.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
            executable.chmod(0o755)
            self.assertTrue(urdf_requires_xacro(source))
            plan = plan_xacro_expansion(
                source,
                root / "cache",
                ros_distro="jazzy",
                explicit_executable=executable,
                environment={"PATH": ""},
            )
            self.assertTrue(plan.requires_expansion)
            self.assertEqual(plan.command, (str(executable.resolve()), str(source.resolve()), "namespace:="))
            self.assertEqual(plan.output.name, "robot.expanded.urdf")

    def test_xacro_plan_fails_clearly_when_explicit_binary_is_missing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "robot.urdf"
            source.write_text("<robot><link name='${namespace}base'/></robot>", encoding="utf-8")
            with self.assertRaisesRegex(FileNotFoundError, "contains xacro syntax"):
                plan_xacro_expansion(
                    source,
                    root / "cache",
                    ros_distro="jazzy",
                    explicit_executable=root / "missing-xacro",
                    environment={"PATH": ""},
                )

    def test_commented_xacro_is_not_treated_as_unresolved_syntax(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / "expanded.urdf"
            source.write_text(
                "<robot name='ready'><!-- <xacro:include filename='old.xacro'/> ${ignored} --></robot>",
                encoding="utf-8",
            )
            self.assertFalse(urdf_requires_xacro(source))

    def test_invalid_newer_xacro_cache_is_regenerated(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            source = root / "robot.urdf"
            source.write_text("<robot><link name='${namespace}base'/></robot>", encoding="utf-8")
            executable = root / "xacro"
            executable.write_text("#!/bin/sh\nprintf \"<robot name='expanded'/>\\n\"\n", encoding="utf-8")
            executable.chmod(0o755)
            cached = root / "cache" / "robot" / "robot.expanded.urdf"
            cached.parent.mkdir(parents=True)
            cached.write_text("<robot><link name='${still_bad}'/></robot>", encoding="utf-8")
            newer = source.stat().st_mtime + 10.0
            os.utime(cached, (newer, newer))
            output = prepare_importable_urdf(
                source,
                root / "cache",
                ros_distro="jazzy",
                explicit_executable=executable,
                environment={"PATH": ""},
            )
            self.assertEqual(output.read_text(encoding="utf-8"), "<robot name='expanded'/>\n")


if __name__ == "__main__":
    unittest.main()
