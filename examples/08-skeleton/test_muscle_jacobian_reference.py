#!/usr/bin/env python3

from __future__ import annotations

import importlib
import math
import sys
import unittest
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
REFERENCE_DIR = SCRIPT_DIR / "reference"
MUSCLE_XML = SCRIPT_DIR / "muscles.xml"
HUMAN_URDF = SCRIPT_DIR / "models" / "Rajagopal2015.urdf"

ANGLE_UNIT_EPSILON = 1e-6
RMSE_TOLERANCE = 6.0e-2
MAX_ABS_ERROR_TOLERANCE = 3.5e-1
WORST_MISMATCH_COUNT = 5

COORDINATE_TO_JOINT = {
    "/jointset/ankle_r/ankle_angle_r/value": "ankle_angle_r",
    "/jointset/hip_r/hip_adduction_r/value": "hip_adduction_r",
    "/jointset/hip_r/hip_flexion_r/value": "hip_flexion_r",
    "/jointset/hip_r/hip_rotation_r/value": "hip_rotation_r",
    "/jointset/knee_r/knee_angle_r/value": "knee_angle_r",
    "/jointset/walker_knee_r/knee_angle_r/value": "knee_angle_r",
    "/jointset/mtp_r/mtp_angle_r/value": "mtp_angle_r",
    "/jointset/subtalar_r/subtalar_angle_r/value": "subtalar_angle_r",
}


@dataclass(frozen=True)
class ReferenceSample:
    time: float
    angle_raw: float
    values: list[float]


@dataclass(frozen=True)
class ReferenceTable:
    path: Path
    coordinate_path: str
    muscle_names: list[str]
    samples: list[ReferenceSample]
    metadata: dict[str, str]


@dataclass(frozen=True)
class ResolvedReferenceColumn:
    reference_name: str
    row_index: int
    table_index: int


@dataclass(frozen=True)
class Mismatch:
    abs_error: float
    reference_name: str
    time: float
    angle_radians: float
    reference_value: float
    model_value: float


@dataclass
class ErrorStats:
    count: int = 0
    sum_sq_error: float = 0.0
    max_abs_error: float = 0.0
    mismatches: list[Mismatch] = field(default_factory=list)

    def add(
        self,
        *,
        reference_name: str,
        time: float,
        angle_radians: float,
        reference_value: float,
        model_value: float,
    ) -> None:
        error = reference_value - model_value
        abs_error = abs(error)
        self.count += 1
        self.sum_sq_error += error * error
        self.max_abs_error = max(self.max_abs_error, abs_error)
        self.mismatches.append(
            Mismatch(
                abs_error=abs_error,
                reference_name=reference_name,
                time=time,
                angle_radians=angle_radians,
                reference_value=reference_value,
                model_value=model_value,
            )
        )

    @property
    def rmse(self) -> float:
        if self.count == 0:
            return 0.0
        return math.sqrt(self.sum_sq_error / self.count)

    def worst(self, limit: int = WORST_MISMATCH_COUNT) -> list[Mismatch]:
        return sorted(self.mismatches, key=lambda item: item.abs_error, reverse=True)[:limit]


def import_sai_model_py():
    try:
        return importlib.import_module("sai_model_py")
    except ImportError:
        candidate_dirs = [
            REPO_ROOT.parent / "sai-model" / "build" / "python",
            REPO_ROOT.parent / "sai-model" / "build-py314" / "python",
        ]
        for candidate_dir in candidate_dirs:
            if not candidate_dir.exists():
                continue
            candidate = str(candidate_dir.resolve())
            if candidate not in sys.path:
                sys.path.insert(0, candidate)
            try:
                return importlib.import_module("sai_model_py")
            except ImportError:
                continue

    raise ImportError(
        "Could not import sai_model_py from PYTHONPATH or sibling sai-model build directories."
    )


def parse_reference_table(path: Path) -> ReferenceTable:
    lines = path.read_text().splitlines()
    try:
        header_end = next(i for i, line in enumerate(lines) if line.strip() == "endheader")
    except StopIteration as exc:
        raise AssertionError(f"{path}: missing endheader") from exc

    metadata: dict[str, str] = {}
    for line in lines[:header_end]:
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        metadata[key.strip()] = value.strip()

    header_columns = lines[header_end + 1].split()
    if len(header_columns) < 3:
        raise AssertionError(f"{path}: expected at least 3 header columns")

    coordinate_path = header_columns[1]
    muscle_names = header_columns[2:]

    samples: list[ReferenceSample] = []
    for line in lines[header_end + 2 :]:
        stripped = line.strip()
        if not stripped:
            continue
        parts = stripped.split()
        if len(parts) != len(header_columns):
            raise AssertionError(
                f"{path}: expected {len(header_columns)} columns, got {len(parts)}"
            )
        samples.append(
            ReferenceSample(
                time=float(parts[0]),
                angle_raw=float(parts[1]),
                values=[float(value) for value in parts[2:]],
            )
        )

    return ReferenceTable(
        path=path,
        coordinate_path=coordinate_path,
        muscle_names=muscle_names,
        samples=samples,
        metadata=metadata,
    )


def parse_muscle_row_map(muscle_xml: Path) -> dict[str, int]:
    root = ET.parse(muscle_xml).getroot()
    row_map: dict[str, int] = {}
    for muscle_node in root.findall("muscleNode"):
        muscle_name = (muscle_node.findtext("muscleName") or "").strip()
        if not muscle_name:
            continue
        if muscle_name in row_map:
            raise AssertionError(f"Duplicate muscle name in XML: {muscle_name}")
        row_map[muscle_name] = len(row_map)
    return row_map


def resolve_reference_columns(
    table: ReferenceTable, row_map: dict[str, int]
) -> tuple[list[ResolvedReferenceColumn], list[str]]:
    resolved: list[ResolvedReferenceColumn] = []
    missing: list[str] = []
    for table_index, reference_name in enumerate(table.muscle_names):
        row_index = row_map.get(reference_name)
        if row_index is None:
            missing.append(reference_name)
            continue
        resolved.append(
            ResolvedReferenceColumn(
                reference_name=reference_name,
                row_index=row_index,
                table_index=table_index,
            )
        )
    return resolved, missing


def normalize_angles(table: ReferenceTable) -> list[float]:
    raw_angles = [sample.angle_raw for sample in table.samples]
    max_abs_angle = max((abs(angle) for angle in raw_angles), default=0.0)
    if max_abs_angle > 2.0 * math.pi + ANGLE_UNIT_EPSILON:
        return [math.radians(angle) for angle in raw_angles]
    return raw_angles


class MuscleJacobianReferenceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.sai_model_py = import_sai_model_py()
        cls.tables = [
            parse_reference_table(path)
            for path in sorted(REFERENCE_DIR.iterdir())
            if path.is_file()
        ]
        if not cls.tables:
            raise AssertionError(f"No reference files found in {REFERENCE_DIR}")

        cls.row_map = parse_muscle_row_map(MUSCLE_XML)

        cls.robot = cls.sai_model_py.SaiModel(str(HUMAN_URDF))
        cls.robot.add_muscle_system(str(MUSCLE_XML), "main")
        cls.joint_names = list(cls.robot.joint_names())
        cls.zero_q = [0.0] * len(cls.joint_names)

        if len(cls.zero_q) != len(cls.robot.q):
            raise AssertionError(
                f"Expected q size {len(cls.joint_names)}, got {len(cls.robot.q)}"
            )
        if cls.robot.get_num_muscles() != len(cls.row_map):
            raise AssertionError(
                "Muscle count mismatch between runtime model and muscles.xml: "
                f"{cls.robot.get_num_muscles()} != {len(cls.row_map)}"
            )

    def test_reference_files_match_named_joints(self) -> None:
        for table in self.tables:
            self.assertIn(
                table.coordinate_path,
                COORDINATE_TO_JOINT,
                msg=f"Unmapped reference coordinate path in {table.path.name}",
            )

    def test_reference_columns_exist_in_muscle_xml(self) -> None:
        for table in self.tables:
            resolved, missing = resolve_reference_columns(table, self.row_map)
            self.assertGreater(
                len(resolved), 0, msg=f"No reference muscle names mapped for {table.path.name}"
            )
            self.assertEqual(
                missing,
                [],
                msg=f"Reference muscles missing from muscles.xml in {table.path.name}: {missing}",
            )

    def test_muscle_jacobian_matches_reference_values(self) -> None:
        compared_entries = 0
        diagnostics: list[str] = []

        for table in self.tables:
            joint_name = COORDINATE_TO_JOINT[table.coordinate_path]
            joint_index = self.robot.joint_index(joint_name)
            resolved_columns, missing = resolve_reference_columns(table, self.row_map)
            self.assertEqual(missing, [], msg=f"Missing muscle mappings in {table.path.name}")
            direct_stats = ErrorStats()
            flipped_stats = ErrorStats()

            for sample, q_angle in zip(table.samples, normalize_angles(table)):
                q = list(self.zero_q)
                q[joint_index] = q_angle
                self.robot.q = q
                self.robot.update_model()
                jacobian = self.robot.compute_muscle_jacobian(floating=False)

                for column in resolved_columns:
                    reference_value = sample.values[column.table_index]
                    model_value = float(jacobian[column.row_index, joint_index])
                    compared_entries += 1
                    direct_stats.add(
                        reference_name=column.reference_name,
                        time=sample.time,
                        angle_radians=q_angle,
                        reference_value=reference_value,
                        model_value=model_value,
                    )
                    flipped_stats.add(
                        reference_name=column.reference_name,
                        time=sample.time,
                        angle_radians=q_angle,
                        reference_value=reference_value,
                        model_value=-model_value,
                    )

            best_label = "L"
            best_stats = direct_stats
            if flipped_stats.rmse < direct_stats.rmse:
                best_label = "-L"
                best_stats = flipped_stats

            diagnostics.append(
                f"{table.path.name}: joint={joint_name}, entries={best_stats.count}, "
                f"best_sign={best_label}, rmse={best_stats.rmse:.6f}, "
                f"max_abs_error={best_stats.max_abs_error:.6f}"
            )
            for mismatch in best_stats.worst():
                diagnostics.append(
                    "  "
                    f"{mismatch.reference_name} at t={mismatch.time:.1f}, "
                    f"q={mismatch.angle_radians:.6f}: "
                    f"ref={mismatch.reference_value:.6f}, "
                    f"model={mismatch.model_value:.6f}, "
                    f"abs={mismatch.abs_error:.6f}"
                )

            self.assertGreater(best_stats.count, 0, msg=f"No entries compared for {table.path.name}")
            self.assertLessEqual(
                best_stats.rmse,
                RMSE_TOLERANCE,
                msg="\n".join(
                    [
                        f"{table.path.name}: RMSE {best_stats.rmse:.6f} exceeds {RMSE_TOLERANCE:.6f}",
                        *diagnostics[-(WORST_MISMATCH_COUNT + 1) :],
                    ]
                ),
            )
            self.assertLessEqual(
                best_stats.max_abs_error,
                MAX_ABS_ERROR_TOLERANCE,
                msg="\n".join(
                    [
                        f"{table.path.name}: max abs error {best_stats.max_abs_error:.6f} exceeds {MAX_ABS_ERROR_TOLERANCE:.6f}",
                        *diagnostics[-(WORST_MISMATCH_COUNT + 1) :],
                    ]
                ),
            )

        self.assertGreater(compared_entries, 0)
        print()
        print("\n".join(diagnostics))


if __name__ == "__main__":
    unittest.main(verbosity=2)
