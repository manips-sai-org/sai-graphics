#!/usr/bin/env python3

from __future__ import annotations

import importlib
import json
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
HUMAN_URDF = SCRIPT_DIR / "./models/Rajagopal2015.urdf"
OPENSIM_MUSCLE_MAP = SCRIPT_DIR / "archive" / "opensim_muscle_map.json"
ANGLE_UNIT_EPSILON = 1e-6
WORST_MISMATCH_COUNT = 5

COORDINATE_TO_JOINT = {
    "/jointset/ankle_r/ankle_angle_r/value": "ankle_angle_r",
    "/jointset/hip_r/hip_adduction_r/value": "hip_adduction_r",
    "/jointset/hip_r/hip_rotation_r/value": "hip_rotation_r",
    "/jointset/knee_r/knee_angle_r/value": "knee_angle_r",
    "/jointset/back/lumbar_extension/value": "lumbar_extension",
    "/jointset/back/lumbar_rotation/value": "lumbar_rotation",
    "/jointset/back/lumbar_bending/value": "lumbar_bending",
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
    sai_name: str
    table_index: int
    row_index: int


@dataclass(frozen=True)
class Mismatch:
    abs_error: float
    reference_name: str
    sai_name: str
    time: float
    angle_raw: float
    angle_radians: float
    reference_value: float
    model_value: float
    error: float


@dataclass
class ConventionStats:
    label: str
    count: int = 0
    sum_sq_error: float = 0.0
    max_abs_error: float = 0.0
    mismatches: list[Mismatch] = field(default_factory=list)

    def add(
        self,
        *,
        reference_name: str,
        sai_name: str,
        time: float,
        angle_raw: float,
        angle_radians: float,
        reference_value: float,
        model_value: float,
        error: float,
    ) -> None:
        abs_error = abs(error)
        self.count += 1
        self.sum_sq_error += error * error
        self.max_abs_error = max(self.max_abs_error, abs_error)
        self.mismatches.append(
            Mismatch(
                abs_error=abs_error,
                reference_name=reference_name,
                sai_name=sai_name,
                time=time,
                angle_raw=angle_raw,
                angle_radians=angle_radians,
                reference_value=reference_value,
                model_value=model_value,
                error=error,
            )
        )

    @property
    def rmse(self) -> float:
        if self.count == 0:
            return 0.0
        return math.sqrt(self.sum_sq_error / self.count)

    def worst(self, limit: int = WORST_MISMATCH_COUNT) -> list[Mismatch]:
        return sorted(self.mismatches, key=lambda mismatch: mismatch.abs_error, reverse=True)[
            :limit
        ]


@dataclass(frozen=True)
class FileDiagnostic:
    table_name: str
    joint_name: str
    angle_rule: str
    angle_raw_unit: str
    matched_columns: list[ResolvedReferenceColumn]
    skipped_columns: list[str]
    direct: ConventionStats
    flipped: ConventionStats

    @property
    def better_sign(self) -> str:
        if self.flipped.rmse < self.direct.rmse:
            return "-L"
        return "L"

    @property
    def compared_entries(self) -> int:
        return self.direct.count

    def best_stats(self) -> ConventionStats:
        if self.better_sign == "-L":
            return self.flipped
        return self.direct


@dataclass(frozen=True)
class DiagnosticRun:
    module_path: str
    muscle_count: int
    joint_count: int
    default_jacobian_joint_count: int
    diagnostics: list[FileDiagnostic]

    def render(self) -> str:
        lines = [
            "Muscle Jacobian reference diagnostic",
            f"  sai_model_py: {self.module_path}",
            f"  Jacobian size: {self.muscle_count} muscles x {self.joint_count} joints",
            "  Default floating Jacobian size: "
            f"{self.muscle_count} muscles x {self.default_jacobian_joint_count} joints",
            f"  Reference tables: {len(self.diagnostics)}",
        ]

        total_entries = sum(diagnostic.compared_entries for diagnostic in self.diagnostics)
        lines.append(f"  Compared entries: {total_entries}")
        lines.append("")

        for diagnostic in self.diagnostics:
            lines.append(f"{diagnostic.table_name}")
            lines.append(f"  joint: {diagnostic.joint_name}")
            lines.append(f"  angle interpretation: {diagnostic.angle_rule}")
            lines.append(
                f"  matched muscles: {len(diagnostic.matched_columns)} / "
                f"{len(diagnostic.matched_columns) + len(diagnostic.skipped_columns)}"
            )
            if diagnostic.skipped_columns:
                skipped = ", ".join(diagnostic.skipped_columns)
                lines.append(
                    f"  skipped muscles ({len(diagnostic.skipped_columns)}): {skipped}"
                )
            lines.append(f"  compared entries: {diagnostic.compared_entries}")
            lines.append(f"  lower-error sign: {diagnostic.better_sign}")
            lines.append(
                "  RMSE: "
                f"L={diagnostic.direct.rmse:.6f}, "
                f"-L={diagnostic.flipped.rmse:.6f}"
            )
            lines.append(
                "  max abs error: "
                f"L={diagnostic.direct.max_abs_error:.6f}, "
                f"-L={diagnostic.flipped.max_abs_error:.6f}"
            )

            best_stats = diagnostic.best_stats()
            worst = best_stats.worst()
            if worst:
                lines.append(f"  worst mismatches for {diagnostic.better_sign}:")
                for mismatch in worst:
                    lines.append(
                        "    "
                        f"{mismatch.reference_name} -> {mismatch.sai_name} "
                        f"(t={mismatch.time:.1f}, raw={mismatch.angle_raw:.6f} "
                        f"{diagnostic.angle_raw_unit}, q={mismatch.angle_radians:.6f} rad): "
                        f"ref={mismatch.reference_value:.6f}, "
                        f"model={mismatch.model_value:.6f}, "
                        f"abs={mismatch.abs_error:.6f}"
                    )
            lines.append("")

        return "\n".join(lines).rstrip()


def import_sai_model_py():
    try:
        return importlib.import_module("sai_model_py")
    except ImportError as original_exc:
        sibling_build = (REPO_ROOT.parent / "sai-model" / "build" / "python").resolve()
        if str(sibling_build) not in sys.path:
            sys.path.insert(0, str(sibling_build))
        try:
            return importlib.import_module("sai_model_py")
        except ImportError as retry_exc:
            raise ImportError(
                "Could not import sai_model_py from PYTHONPATH or the sibling "
                f"build directory: {sibling_build}"
            ) from retry_exc if sibling_build.exists() else original_exc


def parse_reference_table(path: Path) -> ReferenceTable:
    lines = path.read_text().splitlines()
    try:
        header_end = next(index for index, line in enumerate(lines) if line.strip() == "endheader")
    except StopIteration as exc:
        raise AssertionError(f"{path.name}: missing endheader") from exc

    metadata: dict[str, str] = {}
    for line in lines[:header_end]:
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        metadata[key.strip()] = value.strip()

    header_columns = lines[header_end + 1].split()
    if len(header_columns) < 3:
        raise AssertionError(f"{path.name}: expected at least 3 header columns")

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
                f"{path.name}: expected {len(header_columns)} columns, got {len(parts)}"
            )
        samples.append(
            ReferenceSample(
                time=float(parts[0]),
                angle_raw=float(parts[1]),
                values=[float(value) for value in parts[2:]],
            )
        )

    declared_rows = metadata.get("nRows")
    if declared_rows is not None:
        if int(declared_rows) != len(samples):
            raise AssertionError(
                f"{path.name}: declared {declared_rows} rows but parsed {len(samples)}"
            )

    declared_columns = metadata.get("nColumns")
    if declared_columns is not None:
        if int(declared_columns) != len(header_columns):
            raise AssertionError(
                f"{path.name}: declared {declared_columns} columns but parsed {len(header_columns)}"
            )

    return ReferenceTable(
        path=path,
        coordinate_path=coordinate_path,
        muscle_names=muscle_names,
        samples=samples,
        metadata=metadata,
    )


def parse_muscle_row_map(muscle_xml: Path) -> tuple[list[str], dict[str, int]]:
    root = ET.parse(muscle_xml).getroot()
    muscle_names: list[str] = []
    row_map: dict[str, int] = {}

    for muscle_node in root.findall("muscleNode"):
        name_element = muscle_node.find("muscleName")
        if name_element is None or name_element.text is None:
            continue
        muscle_name = name_element.text.strip()
        if not muscle_name:
            continue
        if muscle_name in row_map:
            raise AssertionError(f"Duplicate active muscle name in XML: {muscle_name}")
        row_map[muscle_name] = len(muscle_names)
        muscle_names.append(muscle_name)

    return muscle_names, row_map


def load_opensim_muscle_map(path: Path) -> dict[str, str]:
    return json.loads(path.read_text())


def normalize_angles(table: ReferenceTable) -> tuple[list[float], str, str]:
    raw_angles = [sample.angle_raw for sample in table.samples]
    max_abs_angle = max((abs(angle) for angle in raw_angles), default=0.0)
    header_in_degrees = table.metadata.get("inDegrees", "unknown")
    if max_abs_angle > (2.0 * math.pi + ANGLE_UNIT_EPSILON):
        return (
            [math.radians(angle) for angle in raw_angles],
            f"degrees (heuristic override; header inDegrees={header_in_degrees})",
            "deg",
        )

    return (
        list(raw_angles),
        f"radians (trusted as-is; header inDegrees={header_in_degrees})",
        "rad",
    )


def resolve_reference_columns(
    muscle_names: list[str],
    row_map: dict[str, int],
    alias_map: dict[str, str],
) -> tuple[list[ResolvedReferenceColumn], list[str]]:
    resolved: list[ResolvedReferenceColumn] = []
    skipped: list[str] = []

    for table_index, reference_name in enumerate(muscle_names):
        sai_name = alias_map.get(reference_name, reference_name)
        row_index = row_map.get(sai_name)
        if row_index is None:
            skipped.append(reference_name)
            continue
        resolved.append(
            ResolvedReferenceColumn(
                reference_name=reference_name,
                sai_name=sai_name,
                table_index=table_index,
                row_index=row_index,
            )
        )

    return resolved, skipped


def compare_reference_table(
    robot,
    zero_q: list[float],
    row_map: dict[str, int],
    alias_map: dict[str, str],
    table: ReferenceTable,
) -> FileDiagnostic:
    joint_name = COORDINATE_TO_JOINT.get(table.coordinate_path)
    if joint_name is None:
        raise AssertionError(
            f"{table.path.name}: no joint mapping for coordinate path {table.coordinate_path}"
        )

    joint_index = robot.joint_index(joint_name)
    resolved_columns, skipped_columns = resolve_reference_columns(
        table.muscle_names, row_map, alias_map
    )
    if not resolved_columns:
        raise AssertionError(f"{table.path.name}: no reference muscles mapped into Jacobian rows")

    angle_radians, angle_rule, angle_raw_unit = normalize_angles(table)
    direct = ConventionStats("L")
    flipped = ConventionStats("-L")

    for sample, q_angle in zip(table.samples, angle_radians):
        q = list(zero_q)
        q[joint_index] = q_angle
        robot.q = q
        robot.update_model()
        jacobian = robot.compute_muscle_jacobian(floating=False)

        for column in resolved_columns:
            reference_value = sample.values[column.table_index]
            model_value = float(jacobian[column.row_index, joint_index])
            if not math.isfinite(reference_value):
                raise AssertionError(
                    f"{table.path.name}: non-finite reference value for {column.reference_name}"
                )
            if not math.isfinite(model_value):
                raise AssertionError(
                    f"{table.path.name}: non-finite Jacobian entry for {column.sai_name}"
                )

            direct.add(
                reference_name=column.reference_name,
                sai_name=column.sai_name,
                time=sample.time,
                angle_raw=sample.angle_raw,
                angle_radians=q_angle,
                reference_value=reference_value,
                model_value=model_value,
                error=reference_value - model_value,
            )
            flipped.add(
                reference_name=column.reference_name,
                sai_name=column.sai_name,
                time=sample.time,
                angle_raw=sample.angle_raw,
                angle_radians=q_angle,
                reference_value=reference_value,
                model_value=-model_value,
                error=reference_value + model_value,
            )

    return FileDiagnostic(
        table_name=table.path.name,
        joint_name=joint_name,
        angle_rule=angle_rule,
        angle_raw_unit=angle_raw_unit,
        matched_columns=resolved_columns,
        skipped_columns=skipped_columns,
        direct=direct,
        flipped=flipped,
    )


def run_diagnostic() -> DiagnosticRun:
    sai_model_py = import_sai_model_py()

    tables = [parse_reference_table(path) for path in sorted(REFERENCE_DIR.iterdir()) if path.is_file()]
    if not tables:
        raise AssertionError(f"No reference tables found in {REFERENCE_DIR}")

    muscle_names, row_map = parse_muscle_row_map(MUSCLE_XML)
    alias_map = load_opensim_muscle_map(OPENSIM_MUSCLE_MAP)

    robot = sai_model_py.SaiModel(str(HUMAN_URDF))
    initial_muscle_count = robot.get_num_muscles()
    if initial_muscle_count != 0:
        raise AssertionError(
            f"Expected zero muscles before loading the XML, got {initial_muscle_count}"
        )
    robot.add_muscle_system(str(MUSCLE_XML), "main")

    runtime_muscle_count = robot.get_num_muscles()
    if runtime_muscle_count != len(muscle_names):
        raise AssertionError(
            "Runtime muscle count disagrees with XML muscle order: "
            f"{runtime_muscle_count} != {len(muscle_names)}"
        )

    jacobian = robot.compute_muscle_jacobian(floating=False)
    default_jacobian = robot.compute_muscle_jacobian()
    joint_names = list(robot.joint_names())
    zero_q = [0.0] * len(joint_names)

    if len(zero_q) != len(robot.q):
        raise AssertionError(
            f"Expected q size {len(joint_names)}, got {len(robot.q)} from SaiModel"
        )
    if jacobian.shape != (runtime_muscle_count, len(joint_names)):
        raise AssertionError(
            "Unexpected Jacobian shape: "
            f"{jacobian.shape} != ({runtime_muscle_count}, {len(joint_names)})"
        )
    expected_default_columns = max(len(joint_names) - 6, 0)
    if default_jacobian.shape != (runtime_muscle_count, expected_default_columns):
        raise AssertionError(
            "Unexpected default floating Jacobian shape: "
            f"{default_jacobian.shape} != ({runtime_muscle_count}, {expected_default_columns})"
        )

    diagnostics = [
        compare_reference_table(robot, zero_q, row_map, alias_map, table) for table in tables
    ]

    return DiagnosticRun(
        module_path=getattr(sai_model_py, "__file__", "<unknown>"),
        muscle_count=runtime_muscle_count,
        joint_count=len(joint_names),
        default_jacobian_joint_count=default_jacobian.shape[1],
        diagnostics=diagnostics,
    )


class MuscleJacobianReferenceTest(unittest.TestCase):
    def test_reference_diagnostic(self) -> None:
        diagnostic = run_diagnostic()

        self.assertGreater(diagnostic.muscle_count, 0)
        self.assertGreater(diagnostic.joint_count, 0)
        self.assertEqual(diagnostic.default_jacobian_joint_count, diagnostic.joint_count - 6)
        self.assertEqual(len(diagnostic.diagnostics), 7)

        for file_diagnostic in diagnostic.diagnostics:
            self.assertIn(file_diagnostic.joint_name, COORDINATE_TO_JOINT.values())
            self.assertGreater(len(file_diagnostic.matched_columns), 0)
            self.assertGreater(file_diagnostic.compared_entries, 0)
            self.assertEqual(file_diagnostic.direct.count, file_diagnostic.flipped.count)
            self.assertTrue(math.isfinite(file_diagnostic.direct.rmse))
            self.assertTrue(math.isfinite(file_diagnostic.flipped.rmse))
            self.assertTrue(math.isfinite(file_diagnostic.direct.max_abs_error))
            self.assertTrue(math.isfinite(file_diagnostic.flipped.max_abs_error))

        print()
        print(diagnostic.render())


if __name__ == "__main__":
    unittest.main(verbosity=2)
