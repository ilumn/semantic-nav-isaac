import math
import unittest
from pathlib import Path

from semantic_nav_isaac_scene.geometry import (
    fixed_parent_local_linear_velocity,
    fixed_parent_relative_translation,
    horizontal_focal_length_mm,
    pose_is_inside_xy,
    quaternion_to_matrix,
    rotate_vector,
    rpy_to_quaternion,
    warehouse_interior_bounds,
)
from semantic_nav_isaac_scene.manifest import load_manifest


SCENE_ROOT = Path(__file__).resolve().parent.parent


class GeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = load_manifest(SCENE_ROOT / "config" / "scene_manifest.json")

    def test_wall_boxes_enclose_exact_four_by_six_meter_room(self):
        walls = self.manifest["stage"]["warehouse"]["walls"]
        self.assertEqual(warehouse_interior_bounds(walls), (-2.0, 2.0, -3.0, 3.0))

    def test_robot_and_enabled_targets_start_inside_room(self):
        bounds = warehouse_interior_bounds(self.manifest["stage"]["warehouse"]["walls"])
        self.assertTrue(pose_is_inside_xy(self.manifest["robot"]["spawn"]["xyz"], bounds, margin=0.1))
        for target in self.manifest["semantic_targets"]:
            if target["enabled"]:
                self.assertTrue(pose_is_inside_xy(target["pose"]["xyz"], bounds, margin=0.1), target["id"])

    def test_converted_semantic_assets_map_y_up_to_stage_z_up(self):
        targets = {target["id"]: target for target in self.manifest["semantic_targets"]}
        expected_rolls = {
            "table": math.pi / 2.0,
            "person": math.pi / 2.0 + 0.04,
            "stop_sign": math.pi / 2.0,
        }
        for target_id, expected_roll in expected_rolls.items():
            rpy = targets[target_id]["asset_pose"]["rpy"]
            self.assertAlmostEqual(rpy[0], expected_roll, places=12, msg=target_id)
            self.assertEqual(rpy[1:], [0.0, 0.0], target_id)
            transformed_up = rotate_vector(rpy_to_quaternion(rpy), (0.0, 1.0, 0.0))
            self.assertGreater(transformed_up[2], 0.99, target_id)

    def test_camera_optical_rotation_is_180_degrees_about_x(self):
        quaternion = self.manifest["sensors"]["camera"]["usd_camera_to_ros_optical_wxyz"]
        matrix = quaternion_to_matrix(quaternion)
        expected = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, -1.0))
        for row, expected_row in zip(matrix, expected):
            for value, expected_value in zip(row, expected_row):
                self.assertAlmostEqual(value, expected_value)

    def test_focal_length_matches_declared_horizontal_fov(self):
        camera = self.manifest["sensors"]["camera"]
        focal = horizontal_focal_length_mm(camera["horizontal_aperture_mm"], camera["horizontal_fov_rad"])
        reconstructed_fov = 2.0 * math.atan(camera["horizontal_aperture_mm"] / (2.0 * focal))
        self.assertAlmostEqual(reconstructed_fov, camera["horizontal_fov_rad"])

    def test_relative_odometry_compensates_fixed_parent_offset(self):
        offset = (1.0, 0.0, 0.0)
        self.assertEqual(
            fixed_parent_relative_translation(
                (0.0, 0.0, 0.0),
                (1.0, 0.0, 0.0, 0.0),
                offset,
            ),
            (0.0, 0.0, 0.0),
        )
        shifted = fixed_parent_relative_translation(
            (2.0, 1.0, 0.0),
            rpy_to_quaternion((0.0, 0.0, math.pi / 2.0)),
            offset,
        )
        for value, expected in zip(shifted, (3.0, 0.0, 0.0)):
            self.assertAlmostEqual(value, expected)

    def test_relative_twist_compensates_fixed_parent_offset(self):
        corrected = fixed_parent_local_linear_velocity(
            (0.0, 2.0, 0.0),
            (0.0, 0.0, 2.0),
            (1.0, 0.0, 0.0),
        )
        self.assertEqual(corrected, (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
