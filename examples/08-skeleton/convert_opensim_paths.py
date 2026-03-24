#!/usr/bin/env python3

import argparse
import copy
import json
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple


@dataclass
class PathWaypoint:
    frame: str
    point: Tuple[float, float, float]
    source_type: str


def _text(element: Optional[ET.Element]) -> Optional[str]:
    if element is None or element.text is None:
        return None
    value = element.text.strip()
    return value if value else None


def _parse_vec3(text: str) -> Tuple[float, float, float]:
    parts = text.replace(",", " ").split()
    if len(parts) != 3:
        raise ValueError(f"Expected 3 values, got: {text!r}")
    return tuple(float(value) for value in parts)


def _frame_leaf(name: str) -> str:
    return name.split("/")[-1]


def _find_muscle(root: ET.Element, muscle_name: str) -> ET.Element:
    for element in root.iter():
        if element.get("name") != muscle_name:
            continue
        if element.find(".//GeometryPath") is not None:
            return element
    raise ValueError(f"Could not find OpenSim muscle with GeometryPath: {muscle_name}")


def _waypoint_location(point_el: ET.Element) -> Optional[Tuple[float, float, float]]:
    location = _text(point_el.find("location"))
    if location is not None:
        return _parse_vec3(location)

    xyz = []
    for axis in ("x_location", "y_location", "z_location"):
        axis_el = point_el.find(axis)
        if axis_el is None:
            return None
        value = _text(axis_el.find("Constant/value"))
        if value is None:
            value = _text(axis_el.find("value"))
        if value is None:
            return None
        xyz.append(float(value))
    return (xyz[0], xyz[1], xyz[2])


def extract_opensim_path(
    osim_path: Path, muscle_name: str, body_map: Dict[str, str]
) -> Tuple[List[PathWaypoint], List[str]]:
    tree = ET.parse(osim_path)
    root = tree.getroot()
    muscle_el = _find_muscle(root, muscle_name)
    point_set = muscle_el.find(".//GeometryPath/PathPointSet/objects")
    if point_set is None:
        raise ValueError(f"No PathPointSet found for muscle: {muscle_name}")

    waypoints: List[PathWaypoint] = []
    warnings: List[str] = []
    for point_el in list(point_set):
        source_type = point_el.tag
        frame = (
            _text(point_el.find("socket_parent_frame"))
            or _text(point_el.find("parent_frame"))
            or _text(point_el.find("body"))
        )
        if frame is None:
            warnings.append(
                f"Skipping {source_type} in {muscle_name}: missing parent frame"
            )
            continue

        location = _waypoint_location(point_el)
        if location is None:
            warnings.append(
                f"Skipping {source_type} in {muscle_name}: no constant 3D location found"
            )
            continue

        frame_key = _frame_leaf(frame)
        mapped_frame = body_map.get(frame_key, frame_key)
        waypoints.append(PathWaypoint(mapped_frame, location, source_type))

    if not waypoints:
        raise ValueError(f"No usable waypoints extracted for muscle: {muscle_name}")

    return waypoints, warnings


def format_sai_path_block(waypoints: List[PathWaypoint], indent: str = "") -> str:
    lines = [f"{indent}<muscleTendonPath>"]
    for waypoint in waypoints:
        x, y, z = waypoint.point
        lines.append(f"{indent}<linkName>{waypoint.frame}</linkName>")
        lines.append(f"{indent}<point>{x:.4f}, {y:.4f}, {z:.4f}</point>")
    lines.append(f"{indent}</muscleTendonPath>")
    return "\n".join(lines)


def replace_muscle_path_block(
    sai_path: Path, muscle_name: str, replacement_block: str, output_path: Path
) -> None:
    tree = ET.parse(sai_path)
    root = tree.getroot()

    target_node = None
    for muscle_node in root.findall("muscleNode"):
        name_el = muscle_node.find("muscleName")
        if name_el is not None and _text(name_el) == muscle_name:
            target_node = muscle_node
            break

    if target_node is None:
        raise ValueError(f"Could not find muscle in SAI XML: {muscle_name}")

    contractor = target_node.find("contractorNode")
    if contractor is None:
        raise ValueError(f"Muscle is missing contractorNode: {muscle_name}")

    old_path = contractor.find("muscleTendonPath")
    if old_path is None:
        raise ValueError(f"Muscle is missing muscleTendonPath: {muscle_name}")

    replacement_el = ET.fromstring(replacement_block)
    children = list(contractor)
    insert_index = children.index(old_path)
    contractor.remove(old_path)
    contractor.insert(insert_index, replacement_el)
    ET.indent(tree, space="    ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def load_body_map(path: Optional[Path]) -> Dict[str, str]:
    if path is None:
        return {}
    with path.open() as handle:
        return json.load(handle)


def load_name_map(path: Optional[Path]) -> Dict[str, str]:
    if path is None:
        return {}
    with path.open() as handle:
        return json.load(handle)


def find_sai_muscle_nodes(root: ET.Element) -> Dict[str, ET.Element]:
    nodes = {}
    for muscle_node in root.findall("muscleNode"):
        name_el = muscle_node.find("muscleName")
        name = _text(name_el)
        if name is not None and name not in nodes:
            nodes[name] = muscle_node
    return nodes


def batch_convert_to_sai(
    osim_path: Path,
    sai_template: Path,
    output_path: Path,
    body_map: Dict[str, str],
    muscle_name_map: Dict[str, str],
) -> Tuple[int, List[str]]:
    osim_root = ET.parse(osim_path).getroot()
    sai_tree = ET.parse(sai_template)
    sai_root = sai_tree.getroot()
    sai_nodes = find_sai_muscle_nodes(sai_root)

    converted = 0
    warnings: List[str] = []
    for element in osim_root.iter():
        osim_name = element.get("name")
        if osim_name is None or element.find(".//GeometryPath") is None:
            continue
        target_name = muscle_name_map.get(osim_name, osim_name)
        target_node = sai_nodes.get(target_name)
        if target_node is None:
            continue

        waypoints, extraction_warnings = extract_opensim_path(
            osim_path, osim_name, body_map
        )
        warnings.extend(extraction_warnings)

        contractor = target_node.find("contractorNode")
        if contractor is None:
            warnings.append(f"Skipping {target_name}: missing contractorNode")
            continue
        old_path = contractor.find("muscleTendonPath")
        if old_path is None:
            warnings.append(f"Skipping {target_name}: missing muscleTendonPath")
            continue

        replacement_el = ET.fromstring(format_sai_path_block(waypoints))
        children = list(contractor)
        insert_index = children.index(old_path)
        contractor.remove(old_path)
        contractor.insert(insert_index, copy.deepcopy(replacement_el))
        converted += 1

    ET.indent(sai_tree, space="    ")
    sai_tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return converted, warnings


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Extract a muscle GeometryPath from an OpenSim .osim model and convert it to SAI muscle XML waypoints."
    )
    parser.add_argument("--osim", type=Path, required=True, help="Path to the OpenSim .osim file")
    parser.add_argument("--muscle", help="OpenSim muscle name to extract")
    parser.add_argument(
        "--body-map",
        type=Path,
        help="JSON file mapping OpenSim frame names to SAI/URDF link names",
    )
    parser.add_argument(
        "--muscle-map",
        type=Path,
        help="JSON file mapping OpenSim muscle names to SAI muscle names",
    )
    parser.add_argument(
        "--print-only",
        action="store_true",
        help="Print the converted <muscleTendonPath> block and exit",
    )
    parser.add_argument(
        "--sai-muscles",
        type=Path,
        help="Existing SAI muscle XML file to patch",
    )
    parser.add_argument(
        "--sai-muscle-name",
        help="Target muscle name in the SAI XML. Defaults to --muscle",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output file for the patched SAI XML. Defaults to overwriting --sai-muscles",
    )
    parser.add_argument(
        "--batch-from-template",
        action="store_true",
        help="Use --sai-muscles as a template and replace every matched muscle path into --output",
    )
    args = parser.parse_args()

    body_map = load_body_map(args.body_map)
    muscle_name_map = load_name_map(args.muscle_map)

    if args.batch_from_template:
        if args.sai_muscles is None:
            raise SystemExit("--batch-from-template requires --sai-muscles")
        if args.output is None:
            raise SystemExit("--batch-from-template requires --output")
        converted, warnings = batch_convert_to_sai(
            args.osim, args.sai_muscles, args.output, body_map, muscle_name_map
        )
        for warning in warnings:
            print(f"warning: {warning}", file=sys.stderr)
        print(
            f"Converted {converted} matched muscle paths into {args.output}",
            file=sys.stderr,
        )
        return 0

    if args.muscle is None:
        raise SystemExit("--muscle is required unless --batch-from-template is used")

    waypoints, warnings = extract_opensim_path(args.osim, args.muscle, body_map)
    block = format_sai_path_block(waypoints)

    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)

    if args.print_only or args.sai_muscles is None:
        print(block)
        return 0

    target_name = args.sai_muscle_name or args.muscle
    output_path = args.output or args.sai_muscles
    replace_muscle_path_block(args.sai_muscles, target_name, block, output_path)
    print(
        f"Updated {target_name} in {output_path} using OpenSim path from {args.muscle}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
