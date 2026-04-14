#!/usr/bin/env python3

import importlib.util
import math
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
CONVERTER_PATH = SCRIPT_DIR / "convert_opensim_to_urdf.py"
OSIM_PATH = SCRIPT_DIR / "models" / "Rajagopal2015.osim"


def _load_converter():
    spec = importlib.util.spec_from_file_location("convert_opensim_to_urdf", CONVERTER_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def _parse_vec3(text: str) -> tuple[float, float, float]:
    values = tuple(float(value) for value in text.split())
    if len(values) != 3:
        raise ValueError(f"Expected a vec3, got {text!r}")
    return values


def _matmul3(
    lhs: tuple[tuple[float, float, float], ...],
    rhs: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(lhs[i][k] * rhs[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _transpose3(matrix: tuple[tuple[float, float, float], ...]) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[j][i] for j in range(3)) for i in range(3))


def _urdf_rotation_matrix_from_rpy(rpy: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _opensim_body_fixed_xyz_matrix(rpy: tuple[float, float, float]) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cp * cy, -cp * sy, sp),
        (sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp),
        (-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp),
    )


OPEN_SIM_TO_URDF = (
    (1.0, 0.0, 0.0),
    (0.0, 0.0, -1.0),
    (0.0, 1.0, 0.0),
)
URDF_TO_OPEN_SIM = _transpose3(OPEN_SIM_TO_URDF)


class OpenSimToUrdfConversionTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.converter = _load_converter()
        cls.tmpdir = tempfile.TemporaryDirectory()
        cls.output_path = Path(cls.tmpdir.name) / "Rajagopal2015.urdf"
        cls.converter.convert_osim_to_urdf(OSIM_PATH, cls.output_path)
        cls.urdf_root = ET.parse(cls.output_path).getroot()
        cls.osim_root = ET.parse(OSIM_PATH).getroot()

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmpdir.cleanup()

    def _urdf_axis(self, joint_name: str) -> tuple[float, float, float]:
        joint_el = self.urdf_root.find(f"./joint[@name='{joint_name}']")
        self.assertIsNotNone(joint_el, f"Missing URDF joint {joint_name}")
        axis_el = joint_el.find("axis")
        self.assertIsNotNone(axis_el, f"Missing axis on URDF joint {joint_name}")
        return _parse_vec3(axis_el.attrib["xyz"])

    def _urdf_origin_rpy(self, joint_name: str) -> tuple[float, float, float]:
        joint_el = self.urdf_root.find(f"./joint[@name='{joint_name}']")
        self.assertIsNotNone(joint_el, f"Missing URDF joint {joint_name}")
        origin_el = joint_el.find("origin")
        self.assertIsNotNone(origin_el, f"Missing origin on URDF joint {joint_name}")
        return _parse_vec3(origin_el.attrib["rpy"])

    def _urdf_mesh_filenames(self) -> list[str]:
        return [
            mesh_el.attrib["filename"]
            for mesh_el in self.urdf_root.findall(".//visual/geometry/mesh")
        ]

    def _osim_axis(self, coordinate_name: str) -> tuple[float, float, float]:
        for joint_el in self.osim_root.findall(".//Joint/*"):
            if joint_el.tag == "PinJoint":
                coord_el = joint_el.find("CoordinateSet/objects/Coordinate")
                if coord_el is not None and coord_el.get("name") == coordinate_name:
                    return (0.0, 0.0, 1.0)
                continue

            spatial_transform = joint_el.find("SpatialTransform")
            if spatial_transform is None:
                continue
            for axis_el in spatial_transform.findall("TransformAxis"):
                if (axis_el.findtext("coordinates") or "").strip() == coordinate_name:
                    axis = _parse_vec3((axis_el.findtext("axis") or "0 0 0").strip())
                    coeffs = (
                        axis_el.findtext("function/LinearFunction/coefficients") or ""
                    ).strip().split()
                    if coeffs and float(coeffs[0]) < 0.0:
                        return (-axis[0], -axis[1], -axis[2])
                    return axis

        raise AssertionError(f"Missing OpenSim transform axis for coordinate {coordinate_name}")

    def _osim_orientation_in_parent(self, joint_name: str) -> tuple[float, float, float]:
        joint_el = self.osim_root.find(f".//Joint/*[@name='{joint_name}']")
        self.assertIsNotNone(joint_el, f"Missing OpenSim joint {joint_name}")
        return _parse_vec3((joint_el.findtext("orientation_in_parent") or "0 0 0").strip())

    def test_pin_joint_axes_remain_local_to_joint_frame(self) -> None:
        for joint_name in [
            "ankle_angle_r",
            "subtalar_angle_r",
            "subtalar_angle_l",
            "mtp_angle_r",
            "pro_sup_r",
            "elbow_flex_l",
        ]:
            self.assertEqual(self._urdf_axis(joint_name), (0.0, 1.0, 0.0))

    def test_custom_joint_axes_match_opensim_spatial_transform_axes(self) -> None:
        for coordinate_name in [
            "pelvis_tilt",
            "pelvis_rotation",
            "hip_flexion_r",
            "arm_flex_l",
            "arm_rot_l",
        ]:
            self.assertEqual(self._urdf_axis(coordinate_name), self._osim_axis(coordinate_name))

    def test_export_policy_overrides_match_reference_urdf(self) -> None:
        self.assertEqual(self._urdf_axis("lumbar_extension"), (0.0, 1.0, 0.0))
        self.assertEqual(self._urdf_axis("lumbar_rotation"), (0.0, 0.0, 1.0))
        self.assertEqual(self._urdf_axis("wrist_dev_r"), (0.0, 0.0, 1.0))
        self.assertEqual(self._urdf_axis("wrist_dev_l"), (0.0, 0.0, 1.0))

    def test_exported_mesh_paths_use_obj_geometry_directory(self) -> None:
        mesh_filenames = self._urdf_mesh_filenames()
        self.assertTrue(mesh_filenames)
        self.assertTrue(all(filename.startswith("Geometry/") for filename in mesh_filenames))
        self.assertTrue(all(filename.endswith(".obj") for filename in mesh_filenames))

    def test_mujoco_compiler_block_is_present(self) -> None:
        compiler_el = self.urdf_root.find("./mujoco/compiler")
        self.assertIsNotNone(compiler_el)
        self.assertEqual(compiler_el.attrib["meshdir"], "./Geometry")

    def test_joint_frame_orientation_matches_opensim_orientation_in_parent(self) -> None:
        joint_map = {
            "subtalar_angle_r": "subtalar_r",
            "subtalar_angle_l": "subtalar_l",
            "ankle_angle_r": "ankle_r",
            "pro_sup_r": "radioulnar_r",
        }
        for joint_name, osim_joint_name in joint_map.items():
            urdf_rotation = _urdf_rotation_matrix_from_rpy(self._urdf_origin_rpy(joint_name))
            back_in_opensim_basis = _matmul3(
                _matmul3(URDF_TO_OPEN_SIM, urdf_rotation),
                OPEN_SIM_TO_URDF,
            )
            osim_rotation = _opensim_body_fixed_xyz_matrix(
                self._osim_orientation_in_parent(osim_joint_name)
            )
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(
                        back_in_opensim_basis[i][j],
                        osim_rotation[i][j],
                        places=7,
                        msg=f"{joint_name} rotation mismatch at ({i}, {j})",
                    )


if __name__ == "__main__":
    unittest.main()
