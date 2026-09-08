import tempfile
import unittest
from pathlib import Path

from semantic_nav_isaac_scene.conversion import (
    intermediate_asset_fingerprint,
    intermediate_dependency_files,
    prepare_obj_material_dependencies,
    require_enabled_intermediates,
    unresolved_obj_material_dependencies,
)
from semantic_nav_isaac_scene.manifest import load_manifest


SCENE_ROOT = Path(__file__).resolve().parent.parent


class ConversionMaterialTests(unittest.TestCase):
    def _obj_fixture(self, root: Path):
        obj = root / "semantic_assets" / "intermediate" / "object.obj"
        obj.parent.mkdir(parents=True)
        obj.write_text("mtllib object.mtl\nv 0 0 0\n", encoding="utf-8")
        mtl = obj.with_suffix(".mtl")
        mtl.write_text("newmtl material\nmap_Kd ../materials/textures/diffuse.png\n", encoding="utf-8")
        return obj, mtl

    def test_missing_texture_is_reported_and_then_resolves(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            obj, _ = self._obj_fixture(root)
            problems = unresolved_obj_material_dependencies(obj)
            self.assertEqual(len(problems), 1)
            texture = root / "semantic_assets" / "materials" / "textures" / "diffuse.png"
            texture.parent.mkdir(parents=True)
            texture.write_bytes(b"png")
            self.assertEqual(unresolved_obj_material_dependencies(obj), [])

    def test_prepare_copies_textures_and_applies_stop_sign_override(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            model = root / "model"
            dae = model / "meshes" / "stop.dae"
            dae.parent.mkdir(parents=True)
            dae.write_text("<COLLADA/>", encoding="utf-8")
            source_texture = model / "materials" / "textures" / "StopSign_Diffuse.png"
            source_texture.parent.mkdir(parents=True)
            source_texture.write_bytes(b"diffuse")
            obj, mtl = self._obj_fixture(root / "generated")
            target = {
                "id": "stop_sign",
                "material_overrides": {"map_Kd": "../materials/textures/StopSign_Diffuse.png"},
            }
            prepare_obj_material_dependencies(dae, obj, target)
            self.assertIn("map_Kd ../materials/textures/StopSign_Diffuse.png", mtl.read_text(encoding="utf-8"))
            self.assertEqual(unresolved_obj_material_dependencies(obj, target), [])

    def test_enabled_targets_are_hard_preflight_requirements(self):
        manifest = load_manifest(SCENE_ROOT / "config" / "scene_manifest.json")
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(FileNotFoundError, "table, person"):
                require_enabled_intermediates(manifest, Path(directory))

    def test_usd_cache_fingerprint_covers_obj_mtl_and_texture_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            obj, mtl = self._obj_fixture(root)
            texture = root / "semantic_assets" / "materials" / "textures" / "diffuse.png"
            texture.parent.mkdir(parents=True)
            texture.write_bytes(b"texture-v1")
            dependencies = intermediate_dependency_files(obj)
            self.assertEqual(set(dependencies), {obj.resolve(), mtl.resolve(), texture.resolve()})

            baseline = intermediate_asset_fingerprint(obj)
            texture.write_bytes(b"texture-v2")
            after_texture = intermediate_asset_fingerprint(obj)
            self.assertNotEqual(baseline, after_texture)
            mtl.write_text(
                "newmtl material\nKd 0.5 0.5 0.5\nmap_Kd ../materials/textures/diffuse.png\n",
                encoding="utf-8",
            )
            after_mtl = intermediate_asset_fingerprint(obj)
            self.assertNotEqual(after_texture, after_mtl)
            obj.write_text("mtllib object.mtl\nv 1 0 0\n", encoding="utf-8")
            self.assertNotEqual(after_mtl, intermediate_asset_fingerprint(obj))


if __name__ == "__main__":
    unittest.main()
