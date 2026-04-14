#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from copy import deepcopy
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_OSIM = SCRIPT_DIR / "models" / "Rajagopal2015.osim"
DEFAULT_TEMPLATE = SCRIPT_DIR / "muscles_fixed.xml"
DEFAULT_OUTPUT = SCRIPT_DIR / "muscles.xml"

ACT_TIME_CONSTANT = "0.010"
DEACT_TIME_CONSTANT = "0.050"


def rotate_opensim_point_to_urdf(point: tuple[float, float, float]) -> tuple[float, float, float]:
    x, y, z = point
    return (x, -z, y)


def parse_vec3(text: str) -> tuple[float, float, float]:
    values = tuple(float(value) for value in text.replace(",", " ").split())
    if len(values) != 3:
        raise ValueError(f"Expected vec3, got {text!r}")
    return values


def format_point(point: tuple[float, float, float]) -> str:
    return ", ".join(f"{value:.8g}" for value in point)


def format_scalar(value: float) -> str:
    return f"{value:.8g}"


def build_muscle_node(muscle_el: ET.Element, index: int) -> ET.Element:
    name = muscle_el.attrib["name"]
    node = ET.Element("muscleNode")

    ET.SubElement(node, "muscleName").text = name

    wrap_count = len(muscle_el.findall("./GeometryPath/PathWrapSet/objects/PathWrap"))
    comment = f"Generated from OpenSim muscle '{name}'."
    if wrap_count:
        comment += f" {wrap_count} wrap object(s) were not recreated in SAI."
    ET.SubElement(node, "comments").text = comment

    ET.SubElement(node, "ID").text = str(index)

    activator = ET.SubElement(node, "activatorNode")
    ET.SubElement(activator, "actTimeConstant").text = ACT_TIME_CONSTANT
    ET.SubElement(activator, "deactTimeConstant").text = DEACT_TIME_CONSTANT

    contractor = ET.SubElement(node, "contractorNode")
    path_el = ET.SubElement(contractor, "muscleTendonPath")
    for point_el in muscle_el.findall("./GeometryPath/PathPointSet/objects/PathPoint"):
        body = (point_el.findtext("body") or "").strip()
        location = rotate_opensim_point_to_urdf(parse_vec3(point_el.findtext("location") or "0 0 0"))
        ET.SubElement(path_el, "linkName").text = body
        ET.SubElement(path_el, "point").text = format_point(location)

    optimal_fiber_length = float(muscle_el.findtext("optimal_fiber_length") or 0.0)
    tendon_slack_length = float(muscle_el.findtext("tendon_slack_length") or 0.0)
    max_isometric_force = float(muscle_el.findtext("max_isometric_force") or 0.0)
    pennation_angle = float(muscle_el.findtext("pennation_angle_at_optimal") or 0.0)
    fiber_damping = float(muscle_el.findtext("fiber_damping") or 0.0)
    force_vel_curvature = float(
        muscle_el.findtext("ForceVelocityCurve/concentric_slope_near_vmax") or 0.25
    )

    ET.SubElement(contractor, "optFiberLength").text = format_scalar(optimal_fiber_length)
    ET.SubElement(contractor, "slackTendonLength").text = format_scalar(tendon_slack_length)
    ET.SubElement(contractor, "maxFiberVelocity").text = format_scalar(10.0 * optimal_fiber_length)
    ET.SubElement(contractor, "peakIsoForce").text = format_scalar(max_isometric_force)
    ET.SubElement(contractor, "pennationAngle").text = format_scalar(
        math.degrees(pennation_angle)
    )
    ET.SubElement(contractor, "forceVelCurvature").text = format_scalar(force_vel_curvature)
    ET.SubElement(contractor, "passiveDamping").text = format_scalar(fiber_damping)

    return node


def generate(osim_path: Path, template_path: Path, output_path: Path) -> int:
    template_root = ET.parse(template_path).getroot()
    output_root = ET.Element(template_root.tag, template_root.attrib)

    for child in list(template_root):
        if child.tag == "robotName":
            robot_name_el = ET.SubElement(output_root, "robotName")
            robot_name_el.text = "Rajagopal2015"
            continue
        if child.tag != "muscleNode":
            output_root.append(deepcopy(child))

    output_root.find("comments").text = f"Generated from {osim_path.name}."

    osim_root = ET.parse(osim_path).getroot()
    muscles = osim_root.findall(".//ForceSet/objects/*")
    for index, muscle_el in enumerate(muscles):
        output_root.append(build_muscle_node(muscle_el, index))

    tree = ET.ElementTree(output_root)
    ET.indent(tree, space="    ")
    xml_body = ET.tostring(output_root, encoding="unicode")
    output_path.write_text(
        '<?xml version="1.0"?>\n'
        '<!DOCTYPE SAImuscle SYSTEM "msc.dtd">\n'
        '<!--dynamics: full = full dynamics, stiff = stiff tendons, stnl - stiff tendons and no activator dynamics -->\n'
        f"{xml_body}\n"
    )
    return len(muscles)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate an SAI muscles.xml file from an OpenSim .osim model."
    )
    parser.add_argument("--osim", type=Path, default=DEFAULT_OSIM)
    parser.add_argument("--template", type=Path, default=DEFAULT_TEMPLATE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    count = generate(args.osim, args.template, args.output)
    print(f"Wrote {args.output} with {count} OpenSim muscle definitions.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
