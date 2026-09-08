import copy
import unittest
from pathlib import Path

from semantic_nav_isaac_scene.manifest import (
    ManifestError,
    load_manifest,
    managed_stage_metadata_matches,
    manifest_digest,
    sensor_data_qos_json,
    validate_manifest,
)


SCENE_ROOT = Path(__file__).resolve().parent.parent


class ManifestTests(unittest.TestCase):
    def setUp(self):
        self.manifest = load_manifest(SCENE_ROOT / "config" / "scene_manifest.json")

    def test_contract_is_complete(self):
        self.assertEqual(self.manifest["target"]["isaac_sim"], "6.0.1")
        self.assertEqual(self.manifest["stage"]["physics_hz"], 200)
        self.assertEqual(self.manifest["sensors"]["imu"]["rate_hz"], 200.0)
        self.assertEqual(self.manifest["ros2"]["topics"]["imu"], "/imu")
        self.assertEqual(self.manifest["ros2"]["frames"]["base"], "base_footprint")
        self.assertEqual(self.manifest["ros2"]["tf_ownership"]["isaac"], ["odom->base_footprint"])
        self.assertEqual(
            sensor_data_qos_json(self.manifest),
            '{"history":"keepLast","depth":5,"reliability":"bestEffort",'
            '"durability":"volatile","deadline":0.0,"lifespan":0.0,'
            '"liveliness":"systemDefault","leaseDuration":0.0}',
        )

    def test_rejects_second_odom_parent_for_base_link(self):
        invalid = copy.deepcopy(self.manifest)
        invalid.pop("_manifest_path")
        invalid["ros2"]["tf_ownership"]["isaac"] = ["odom->base_link"]
        with self.assertRaisesRegex(ManifestError, "odom->base_footprint"):
            validate_manifest(invalid)

    def test_rejects_imu_rate_that_does_not_match_physics(self):
        invalid = copy.deepcopy(self.manifest)
        invalid.pop("_manifest_path")
        invalid["sensors"]["imu"]["rate_hz"] = 100.0
        with self.assertRaisesRegex(ManifestError, "must equal"):
            validate_manifest(invalid)

    def test_rejects_nonzero_spawn_rotation_for_relative_odometry(self):
        invalid = copy.deepcopy(self.manifest)
        invalid.pop("_manifest_path")
        invalid["robot"]["spawn"]["rpy"][2] = 0.5
        with self.assertRaisesRegex(ManifestError, "spawn.rpy must be zero"):
            validate_manifest(invalid)

    def test_rejects_nonvertical_fixed_offset_for_planar_odometry(self):
        invalid = copy.deepcopy(self.manifest)
        invalid.pop("_manifest_path")
        invalid["robot"]["base_footprint_to_base_link"]["xyz"][0] = 0.01
        with self.assertRaisesRegex(ManifestError, "Z-only"):
            validate_manifest(invalid)

    def test_rejects_reliable_sensor_qos(self):
        invalid = copy.deepcopy(self.manifest)
        invalid.pop("_manifest_path")
        invalid["ros2"]["sensor_data_qos"]["reliability"] = "reliable"
        with self.assertRaisesRegex(ManifestError, "sensor_data_qos"):
            validate_manifest(invalid)

    def test_managed_stage_metadata_rejects_stale_manifest_digest(self):
        digest = manifest_digest(self.manifest)
        self.assertTrue(
            managed_stage_metadata_matches(
                self.manifest,
                schema_version=1,
                target_version="6.0.1",
                digest=digest,
            )
        )
        changed = copy.deepcopy(self.manifest)
        changed["sensors"]["camera"]["rate_hz"] = 15.0
        self.assertFalse(
            managed_stage_metadata_matches(
                changed,
                schema_version=1,
                target_version="6.0.1",
                digest=digest,
            )
        )


if __name__ == "__main__":
    unittest.main()
