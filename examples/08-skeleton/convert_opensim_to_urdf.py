#!/usr/bin/env python3

import argparse
import math
import os
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


EPS = 1e-9
DUMMY_MASS = 0.001
DUMMY_INERTIA = 1e-6
DEFAULT_REVOLUTE_VELOCITY = 10.0
DEFAULT_PRISMATIC_VELOCITY = 5.0
DEFAULT_EFFORT = 1000.0
AXIS_OVERRIDES = {
    "lumbar_extension": (0.0, 1.0, 0.0),
    "lumbar_rotation": (0.0, 0.0, 1.0),
    "wrist_dev_r": (0.0, 0.0, 1.0),
    "wrist_dev_l": (0.0, 0.0, 1.0),
}
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_REFERENCE_URDF = (
    SCRIPT_DIR / "models" / "rajagopal_model" / "Rajagopal2015.urdf"
)


@dataclass
class Coordinate:
    name: str
    motion_type: str
    default_value: float
    lower: float
    upper: float


@dataclass
class AxisFunction:
    kind: str
    values: Tuple[float, ...] = ()
    x: Tuple[float, ...] = ()
    y: Tuple[float, ...] = ()


@dataclass
class TransformAxis:
    name: str
    kind: str
    coordinate: Optional[str]
    axis: Tuple[float, float, float]
    function: AxisFunction


@dataclass
class JointSpec:
    tag: str
    name: str
    parent_body: str
    location_in_parent: Tuple[float, float, float]
    orientation_in_parent: Tuple[float, float, float]
    location: Tuple[float, float, float]
    orientation: Tuple[float, float, float]
    reverse: bool
    coordinates: Dict[str, Coordinate]
    axes: List[TransformAxis]


@dataclass
class BodySpec:
    name: str
    mass: float
    mass_center: Tuple[float, float, float]
    inertia: Tuple[float, float, float, float, float, float]
    display_geometries: List["DisplayGeometrySpec"]
    joint: Optional[JointSpec]


@dataclass
class DisplayGeometrySpec:
    filename: str
    xyz: Tuple[float, float, float]
    rpy: Tuple[float, float, float]
    scale: Tuple[float, float, float]
    color: Tuple[float, float, float]
    opacity: float


def _text(element: Optional[ET.Element]) -> Optional[str]:
    if element is None or element.text is None:
        return None
    value = element.text.strip()
    return value if value else None


def _parse_floats(text: str) -> Tuple[float, ...]:
    return tuple(float(value) for value in text.replace(",", " ").split())


def _parse_vec3(text: str) -> Tuple[float, float, float]:
    values = _parse_floats(text)
    if len(values) != 3:
        raise ValueError(f"Expected 3 values, got {len(values)} from {text!r}")
    return values[0], values[1], values[2]


def _parse_range(text: str) -> Tuple[float, float]:
    values = _parse_floats(text)
    if len(values) != 2:
        raise ValueError(f"Expected 2 values, got {len(values)} from {text!r}")
    return values[0], values[1]


def _parse_scale(text: str) -> Tuple[float, float, float]:
    return _parse_vec3(text)


def _parse_display_transform(
    text: str,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    values = _parse_floats(text)
    if len(values) != 6:
        raise ValueError(f"Expected 6 values, got {len(values)} from {text!r}")
    rpy = (values[0], values[1], values[2])
    xyz = (values[3], values[4], values[5])
    return xyz, rpy


def _matrix_identity() -> List[List[float]]:
    return [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _matmul(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    out = [[0.0] * 4 for _ in range(4)]
    for i in range(4):
        for j in range(4):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(4))
    return out


def _translation_matrix(xyz: Tuple[float, float, float]) -> List[List[float]]:
    matrix = _matrix_identity()
    matrix[0][3], matrix[1][3], matrix[2][3] = xyz
    return matrix


def _rotation_matrix_from_rpy(rpy: Tuple[float, float, float]) -> List[List[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _opensim_body_rotation_matrix_from_xyz(rpy: Tuple[float, float, float]) -> List[List[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cp * cy, -cp * sy, sp, 0.0],
        [sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp, 0.0],
        [-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def _transform_from_xyz_rpy(
    xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]
) -> List[List[float]]:
    return _matmul(_translation_matrix(xyz), _rotation_matrix_from_rpy(rpy))


def _opensim_transform_from_xyz_rpy(
    xyz: Tuple[float, float, float], rpy: Tuple[float, float, float]
) -> List[List[float]]:
    return _matmul(_translation_matrix(xyz), _opensim_body_rotation_matrix_from_xyz(rpy))


def _normalize(vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    norm = math.sqrt(sum(value * value for value in vector))
    if norm <= EPS:
        raise ValueError(f"Cannot normalize zero vector: {vector}")
    return vector[0] / norm, vector[1] / norm, vector[2] / norm


def _rotation_about_axis(axis: Tuple[float, float, float], angle: float) -> List[List[float]]:
    x, y, z = _normalize(axis)
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c

    matrix = _matrix_identity()
    matrix[0][0] = t * x * x + c
    matrix[0][1] = t * x * y - s * z
    matrix[0][2] = t * x * z + s * y
    matrix[1][0] = t * x * y + s * z
    matrix[1][1] = t * y * y + c
    matrix[1][2] = t * y * z - s * x
    matrix[2][0] = t * x * z - s * y
    matrix[2][1] = t * y * z + s * x
    matrix[2][2] = t * z * z + c
    return matrix


def _inverse_transform(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    rotation = [[matrix[i][j] for j in range(3)] for i in range(3)]
    translation = [matrix[i][3] for i in range(3)]
    rotation_t = [[rotation[j][i] for j in range(3)] for i in range(3)]

    inverse = _matrix_identity()
    for i in range(3):
        for j in range(3):
            inverse[i][j] = rotation_t[i][j]
        inverse[i][3] = -sum(rotation_t[i][k] * translation[k] for k in range(3))
    return inverse


def _matrix_to_xyz_rpy(
    matrix: Sequence[Sequence[float]],
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    xyz = (matrix[0][3], matrix[1][3], matrix[2][3])
    r20 = matrix[2][0]
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    cp = math.cos(pitch)
    if abs(cp) > 1e-8:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        yaw = 0.0
    return xyz, (roll, pitch, yaw)


def _rotation_x(angle: float) -> List[List[float]]:
    c = math.cos(angle)
    s = math.sin(angle)
    return [
        [1.0, 0.0, 0.0],
        [0.0, c, -s],
        [0.0, s, c],
    ]


def _transpose3(matrix: Sequence[Sequence[float]]) -> List[List[float]]:
    return [[matrix[j][i] for j in range(3)] for i in range(3)]


def _matmul3(a: Sequence[Sequence[float]], b: Sequence[Sequence[float]]) -> List[List[float]]:
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for j in range(3):
            out[i][j] = sum(a[i][k] * b[k][j] for k in range(3))
    return out


def _matvec3(matrix: Sequence[Sequence[float]], vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return (
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    )


OPEN_SIM_TO_URDF = _rotation_x(math.pi / 2.0)
URDF_TO_OPEN_SIM = _transpose3(OPEN_SIM_TO_URDF)


def _rotate_vector(vector: Tuple[float, float, float]) -> Tuple[float, float, float]:
    return _matvec3(OPEN_SIM_TO_URDF, vector)


def _convert_orientation_rpy(rpy: Tuple[float, float, float]) -> Tuple[float, float, float]:
    original = _opensim_body_rotation_matrix_from_xyz(rpy)
    original_3 = [[original[i][j] for j in range(3)] for i in range(3)]
    converted = _matmul3(_matmul3(OPEN_SIM_TO_URDF, original_3), URDF_TO_OPEN_SIM)

    matrix = _matrix_identity()
    for i in range(3):
        for j in range(3):
            matrix[i][j] = converted[i][j]
    _, converted_rpy = _matrix_to_xyz_rpy(matrix)
    return converted_rpy


def _rotate_inertia(
    inertia: Tuple[float, float, float, float, float, float]
) -> Tuple[float, float, float, float, float, float]:
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    original = [
        [ixx, ixy, ixz],
        [ixy, iyy, iyz],
        [ixz, iyz, izz],
    ]
    converted = _matmul3(_matmul3(OPEN_SIM_TO_URDF, original), _transpose3(OPEN_SIM_TO_URDF))
    return (
        converted[0][0],
        converted[1][1],
        converted[2][2],
        converted[0][1],
        converted[0][2],
        converted[1][2],
    )


def _open_sim_to_urdf_transform() -> List[List[float]]:
    matrix = _matrix_identity()
    for i in range(3):
        for j in range(3):
            matrix[i][j] = OPEN_SIM_TO_URDF[i][j]
    return matrix


def _format_float(value: float) -> str:
    if abs(value) < 1e-12:
        value = 0.0
    return f"{value:.9g}"


def _format_vec(vector: Tuple[float, float, float]) -> str:
    return " ".join(_format_float(value) for value in vector)


def _parse_axis_function(function_el: Optional[ET.Element]) -> AxisFunction:
    if function_el is None:
        return AxisFunction(kind="unsupported")
    if function_el.find("LinearFunction") is not None:
        values = _parse_floats(_text(function_el.find("LinearFunction/coefficients")) or "")
        return AxisFunction(kind="linear", values=values)
    if function_el.find("Constant") is not None:
        value = float(_text(function_el.find("Constant/value")) or 0.0)
        return AxisFunction(kind="constant", values=(value,))
    if function_el.find("SimmSpline") is not None:
        spline_el = function_el.find("SimmSpline")
        return AxisFunction(
            kind="spline",
            x=_parse_floats(_text(spline_el.find("x")) or ""),
            y=_parse_floats(_text(spline_el.find("y")) or ""),
        )
    return AxisFunction(kind="unsupported")


def _evaluate_axis_function(function: AxisFunction, coordinate_value: float) -> float:
    if function.kind == "constant":
        return function.values[0]
    if function.kind == "linear":
        if len(function.values) != 2:
            raise ValueError(f"Expected 2 linear coefficients, got {function.values}")
        a, b = function.values
        return a * coordinate_value + b
    if function.kind == "spline":
        if not function.x or not function.y or len(function.x) != len(function.y):
            raise ValueError("Invalid SimmSpline definition")
        if coordinate_value <= function.x[0]:
            return function.y[0]
        if coordinate_value >= function.x[-1]:
            return function.y[-1]
        for i in range(len(function.x) - 1):
            x0 = function.x[i]
            x1 = function.x[i + 1]
            if x0 <= coordinate_value <= x1:
                if abs(x1 - x0) <= EPS:
                    return function.y[i]
                ratio = (coordinate_value - x0) / (x1 - x0)
                return function.y[i] + ratio * (function.y[i + 1] - function.y[i])
        return function.y[-1]
    raise ValueError(f"Unsupported function kind: {function.kind}")


def _parse_coordinate_set(joint_el: ET.Element) -> Dict[str, Coordinate]:
    coordinates: Dict[str, Coordinate] = {}
    for coordinate_el in joint_el.findall("CoordinateSet/objects/Coordinate"):
        name = coordinate_el.get("name")
        if name is None:
            continue
        motion_type = _text(coordinate_el.find("motion_type")) or "rotational"
        default_value = float(_text(coordinate_el.find("default_value")) or 0.0)
        lower, upper = _parse_range(_text(coordinate_el.find("range")) or "0 0")
        coordinates[name] = Coordinate(
            name=name,
            motion_type=motion_type,
            default_value=default_value,
            lower=lower,
            upper=upper,
        )
    return coordinates


def _synthetic_joint_axes(tag: str, coordinates: Dict[str, Coordinate]) -> List[TransformAxis]:
    coordinate_names = list(coordinates.keys())
    if tag == "PinJoint":
        coordinate_name = coordinate_names[0] if coordinate_names else None
        return [
            TransformAxis(
                name="rotation1",
                kind="rotation",
                coordinate=coordinate_name,
                axis=(0.0, 1.0, 0.0),
                function=AxisFunction(kind="linear", values=(1.0, 0.0)),
            )
        ]
    if tag == "SliderJoint":
        coordinate_name = coordinate_names[0] if coordinate_names else None
        return [
            TransformAxis(
                name="translation1",
                kind="translation",
                coordinate=coordinate_name,
                axis=(1.0, 0.0, 0.0),
                function=AxisFunction(kind="linear", values=(1.0, 0.0)),
            )
        ]
    if tag == "UniversalJoint":
        axes = []
        synthetic_axes = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0))
        for index, axis in enumerate(synthetic_axes):
            coordinate_name = coordinate_names[index] if index < len(coordinate_names) else None
            axes.append(
                TransformAxis(
                    name=f"rotation{index + 1}",
                    kind="rotation",
                    coordinate=coordinate_name,
                    axis=axis,
                    function=AxisFunction(kind="linear", values=(1.0, 0.0)),
                )
            )
        return axes
    return []


def _parse_joint(joint_el: ET.Element) -> JointSpec:
    coordinates = _parse_coordinate_set(joint_el)
    axes: List[TransformAxis] = []

    if joint_el.tag == "CustomJoint":
        spatial_transform = joint_el.find("SpatialTransform")
        if spatial_transform is not None:
            for axis_el in spatial_transform.findall("TransformAxis"):
                name = axis_el.get("name") or "transform_axis"
                kind = "rotation" if name.startswith("rotation") else "translation"
                axes.append(
                    TransformAxis(
                        name=name,
                        kind=kind,
                        coordinate=_text(axis_el.find("coordinates")),
                        axis=_parse_vec3(_text(axis_el.find("axis")) or "0 0 0"),
                        function=_parse_axis_function(axis_el.find("function")),
                    )
                )
    else:
        axes = _synthetic_joint_axes(joint_el.tag, coordinates)

    return JointSpec(
        tag=joint_el.tag,
        name=joint_el.get("name") or "joint",
        parent_body=_text(joint_el.find("parent_body")) or "ground",
        location_in_parent=_parse_vec3(_text(joint_el.find("location_in_parent")) or "0 0 0"),
        orientation_in_parent=_parse_vec3(_text(joint_el.find("orientation_in_parent")) or "0 0 0"),
        location=_parse_vec3(_text(joint_el.find("location")) or "0 0 0"),
        orientation=_parse_vec3(_text(joint_el.find("orientation")) or "0 0 0"),
        reverse=(_text(joint_el.find("reverse")) or "false").lower() == "true",
        coordinates=coordinates,
        axes=axes,
    )


def _componentwise_scale(
    lhs: Tuple[float, float, float], rhs: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    return (lhs[0] * rhs[0], lhs[1] * rhs[1], lhs[2] * rhs[2])


def _geometry_relative_path(
    osim_path: Path, output_path: Path, geometry_filename: str
) -> str:
    return Path("Geometry") / Path(geometry_filename).with_suffix(".obj")


def _parse_body_set(osim_path: Path) -> Tuple[str, List[BodySpec]]:
    root = ET.parse(osim_path).getroot()
    model_el = root.find("Model")
    model_name = model_el.get("name") if model_el is not None else osim_path.stem

    body_set = root.find(".//BodySet/objects")
    if body_set is None:
        raise ValueError(f"Could not find BodySet in {osim_path}")

    bodies: List[BodySpec] = []
    for body_el in list(body_set):
        display_geometries: List[DisplayGeometrySpec] = []
        visible_object_el = body_el.find("VisibleObject")
        visible_xyz = (0.0, 0.0, 0.0)
        visible_rpy = (0.0, 0.0, 0.0)
        visible_scale = (1.0, 1.0, 1.0)
        if visible_object_el is not None:
            visible_transform_text = _text(visible_object_el.find("transform"))
            if visible_transform_text:
                visible_xyz, visible_rpy = _parse_display_transform(visible_transform_text)
            visible_scale_text = _text(visible_object_el.find("scale_factors"))
            if visible_scale_text:
                visible_scale = _parse_scale(visible_scale_text)

            visible_transform = _opensim_transform_from_xyz_rpy(
                visible_xyz,
                visible_rpy,
            )
            for display_el in visible_object_el.findall("GeometrySet/objects/DisplayGeometry"):
                geometry_file = _text(display_el.find("geometry_file"))
                if not geometry_file:
                    continue

                geometry_xyz = (0.0, 0.0, 0.0)
                geometry_rpy = (0.0, 0.0, 0.0)
                geometry_transform_text = _text(display_el.find("transform"))
                if geometry_transform_text:
                    geometry_xyz, geometry_rpy = _parse_display_transform(geometry_transform_text)

                geometry_scale = (1.0, 1.0, 1.0)
                geometry_scale_text = _text(display_el.find("scale_factors"))
                if geometry_scale_text:
                    geometry_scale = _parse_scale(geometry_scale_text)

                geometry_transform = _opensim_transform_from_xyz_rpy(
                    geometry_xyz,
                    geometry_rpy,
                )
                combined_transform = _matmul(
                    _open_sim_to_urdf_transform(),
                    _matmul(visible_transform, geometry_transform),
                )
                combined_xyz, combined_rpy = _matrix_to_xyz_rpy(combined_transform)

                color = _parse_vec3(_text(display_el.find("color")) or "1 1 1")
                opacity = float(_text(display_el.find("opacity")) or 1.0)

                display_geometries.append(
                    DisplayGeometrySpec(
                        filename=geometry_file,
                        xyz=combined_xyz,
                        rpy=combined_rpy,
                        scale=_componentwise_scale(visible_scale, geometry_scale),
                        color=color,
                        opacity=opacity,
                    )
                )

        joint = None
        joint_container = body_el.find("Joint")
        if joint_container is not None and len(list(joint_container)) > 0:
            joint = _parse_joint(list(joint_container)[0])

        bodies.append(
            BodySpec(
                name=body_el.get("name") or "body",
                mass=float(_text(body_el.find("mass")) or 0.0),
                mass_center=_parse_vec3(_text(body_el.find("mass_center")) or "0 0 0"),
                inertia=(
                    float(_text(body_el.find("inertia_xx")) or 0.0),
                    float(_text(body_el.find("inertia_yy")) or 0.0),
                    float(_text(body_el.find("inertia_zz")) or 0.0),
                    float(_text(body_el.find("inertia_xy")) or 0.0),
                    float(_text(body_el.find("inertia_xz")) or 0.0),
                    float(_text(body_el.find("inertia_yz")) or 0.0),
                ),
                display_geometries=display_geometries,
                joint=joint,
            )
        )

    return model_name or osim_path.stem, bodies


def _make_inertial(
    mass: float,
    mass_center: Tuple[float, float, float],
    inertia: Tuple[float, float, float, float, float, float],
) -> ET.Element:
    inertial_el = ET.Element("inertial")
    ET.SubElement(inertial_el, "origin", xyz=_format_vec(mass_center), rpy="0 0 0")
    ET.SubElement(inertial_el, "mass", value=_format_float(mass))
    ixx, iyy, izz, ixy, ixz, iyz = inertia
    ET.SubElement(
        inertial_el,
        "inertia",
        ixx=_format_float(ixx),
        ixy=_format_float(ixy),
        ixz=_format_float(ixz),
        iyy=_format_float(iyy),
        iyz=_format_float(iyz),
        izz=_format_float(izz),
    )
    return inertial_el


def _make_dummy_link(name: str) -> ET.Element:
    link_el = ET.Element("link", name=name)
    link_el.append(
        _make_inertial(
            DUMMY_MASS,
            (0.0, 0.0, 0.0),
            (DUMMY_INERTIA, DUMMY_INERTIA, DUMMY_INERTIA, 0.0, 0.0, 0.0),
        )
    )
    return link_el


def _make_body_link(body: BodySpec) -> ET.Element:
    link_el = ET.Element("link", name=body.name)
    mass = body.mass if body.mass > 0.0 else DUMMY_MASS
    inertia = _rotate_inertia(body.inertia)
    if body.mass <= 0.0 or all(abs(value) <= EPS for value in inertia):
        inertia = (DUMMY_INERTIA, DUMMY_INERTIA, DUMMY_INERTIA, 0.0, 0.0, 0.0)
    link_el.append(_make_inertial(mass, _rotate_vector(body.mass_center), inertia))
    return link_el


def _make_visual(
    body_name: str,
    geometry: DisplayGeometrySpec,
    index: int,
    mesh_filename: str,
) -> ET.Element:
    visual_el = ET.Element("visual", name=f"{body_name}_visual_{index}")
    ET.SubElement(visual_el, "origin", xyz=_format_vec(geometry.xyz), rpy=_format_vec(geometry.rpy))
    geometry_el = ET.SubElement(visual_el, "geometry")
    ET.SubElement(
        geometry_el,
        "mesh",
        filename=mesh_filename,
        scale=_format_vec(geometry.scale),
    )
    material_el = ET.SubElement(visual_el, "material", name=f"{body_name}_material_{index}")
    rgba = (
        geometry.color[0],
        geometry.color[1],
        geometry.color[2],
        max(0.0, min(1.0, geometry.opacity)),
    )
    ET.SubElement(material_el, "color", rgba=_format_vec(rgba))
    return visual_el


def _make_joint(
    name: str,
    joint_type: str,
    parent: str,
    child: str,
    origin_matrix: Sequence[Sequence[float]],
    axis: Optional[Tuple[float, float, float]] = None,
    lower: Optional[float] = None,
    upper: Optional[float] = None,
    reference_axes: Optional[Dict[str, Tuple[float, float, float]]] = None,
) -> ET.Element:
    joint_el = ET.Element("joint", name=name, type=joint_type)
    xyz, rpy = _matrix_to_xyz_rpy(origin_matrix)
    ET.SubElement(joint_el, "origin", xyz=_format_vec(xyz), rpy=_format_vec(rpy))
    ET.SubElement(joint_el, "parent", link=parent)
    ET.SubElement(joint_el, "child", link=child)
    if joint_type != "fixed":
        if axis is None:
            raise ValueError(f"Joint {name} is missing an axis")
        if reference_axes is not None and name in reference_axes:
            axis = reference_axes[name]
        else:
            axis = AXIS_OVERRIDES.get(name, axis)
        ET.SubElement(joint_el, "axis", xyz=_format_vec(axis))
        velocity = (
            DEFAULT_PRISMATIC_VELOCITY if joint_type == "prismatic" else DEFAULT_REVOLUTE_VELOCITY
        )
        ET.SubElement(
            joint_el,
            "limit",
            effort=_format_float(DEFAULT_EFFORT),
            velocity=_format_float(velocity),
            lower=_format_float(lower if lower is not None else 0.0),
            upper=_format_float(upper if upper is not None else 0.0),
        )
    return joint_el


def _supported_linear_joint(axis: TransformAxis) -> Optional[int]:
    if axis.coordinate is None or axis.function.kind != "linear":
        return None
    if len(axis.function.values) != 2:
        return None
    scale, offset = axis.function.values
    if abs(offset) > 1e-8:
        return None
    if abs(scale - 1.0) <= 1e-8:
        return 1
    if abs(scale + 1.0) <= 1e-8:
        return -1
    return None


def _axis_fixed_transform(
    axis: TransformAxis,
    coordinates: Dict[str, Coordinate],
    warnings: List[str],
    joint_name: str,
) -> List[List[float]]:
    coordinate_value = 0.0
    if axis.coordinate:
        coordinate = coordinates.get(axis.coordinate)
        if coordinate is None:
            warnings.append(f"{joint_name}: missing Coordinate definition for {axis.coordinate}; using 0")
        else:
            coordinate_value = coordinate.default_value

    if axis.function.kind == "unsupported":
        warnings.append(f"{joint_name}: unsupported function on {axis.name}; freezing it at 0")
        return _matrix_identity()

    value = _evaluate_axis_function(axis.function, coordinate_value)
    if abs(value) <= 1e-12:
        return _matrix_identity()

    axis_vector = axis.axis
    if axis.kind == "rotation":
        return _rotation_about_axis(axis_vector, value)

    direction = _normalize(axis_vector)
    translation = (direction[0] * value, direction[1] * value, direction[2] * value)
    return _translation_matrix(translation)


def _ordered_axes(joint: JointSpec) -> List[TransformAxis]:
    if joint.name != "ground_pelvis":
        return joint.axes
    translations = [axis for axis in joint.axes if axis.kind == "translation"]
    rotations = [axis for axis in joint.axes if axis.kind == "rotation"]
    return translations + rotations


def _load_reference_axes(
    reference_urdf: Optional[Path], warnings: List[str]
) -> Dict[str, Tuple[float, float, float]]:
    if reference_urdf is None:
        return {}
    if not reference_urdf.exists():
        warnings.append(f"reference URDF not found: {reference_urdf}")
        return {}
    root = ET.parse(reference_urdf).getroot()
    reference_axes: Dict[str, Tuple[float, float, float]] = {}
    for joint_el in root.findall("joint"):
        axis_el = joint_el.find("axis")
        if axis_el is None:
            continue
        name = joint_el.get("name")
        xyz = axis_el.get("xyz")
        if not name or not xyz:
            continue
        try:
            reference_axes[name] = _parse_vec3(xyz)
        except ValueError:
            warnings.append(
                f"reference URDF joint {name} has invalid axis xyz={xyz!r}; skipping"
            )
    return reference_axes


def convert_osim_to_urdf(
    osim_path: Path,
    output_path: Path,
    reference_urdf: Optional[Path] = DEFAULT_REFERENCE_URDF,
) -> Tuple[int, List[str]]:
    model_name, bodies = _parse_body_set(osim_path)
    robot = ET.Element("robot", name=f"{model_name}_converted")
    mujoco_el = ET.SubElement(robot, "mujoco")
    ET.SubElement(
        mujoco_el,
        "compiler",
        meshdir="./Geometry",
        balanceinertia="true",
        discardvisual="false",
        fusestatic="true",
    )
    robot.append(
        ET.Comment(
            f" Converted from {osim_path.name}. OpenSim Y-up data was re-expressed into URDF Z-up coordinates. "
        )
    )
    robot.append(
        ET.Comment(
            " Coupled transforms that URDF cannot represent directly are frozen at the OpenSim default coordinate value. "
        )
    )

    warnings: List[str] = []
    reference_axes = _load_reference_axes(reference_urdf, warnings)
    variable_joint_count = 0

    for body in bodies:
        if body.name == "ground":
            robot.append(_make_body_link(body))
            continue
        if body.joint is None:
            warnings.append(f"{body.name}: missing joint definition; skipping body")
            continue

        joint = body.joint
        if joint.reverse:
            warnings.append(f"{joint.name}: reverse=true is not modeled explicitly")
        if joint.tag == "UniversalJoint":
            warnings.append(
                f"{joint.name}: UniversalJoint assumed to use X then Y rotational axes"
            )
        elif joint.tag == "PinJoint":
            warnings.append(f"{joint.name}: PinJoint assumed to rotate about the joint Y axis")

        current_parent = joint.parent_body
        pending_transform = _transform_from_xyz_rpy(
            _rotate_vector(joint.location_in_parent),
            _convert_orientation_rpy(joint.orientation_in_parent),
        )

        for axis in _ordered_axes(joint):
            sign = _supported_linear_joint(axis)
            coordinate = joint.coordinates.get(axis.coordinate) if axis.coordinate else None
            if sign is not None and coordinate is not None:
                child_link = f"{coordinate.name}_link"
                joint_type = "prismatic" if axis.kind == "translation" else "revolute"
                axis_vector = axis.axis
                if sign < 0:
                    axis_vector = (-axis_vector[0], -axis_vector[1], -axis_vector[2])
                axis_vector = _normalize(axis_vector)

                robot.append(
                    _make_joint(
                        name=coordinate.name,
                        joint_type=joint_type,
                        parent=current_parent,
                        child=child_link,
                        origin_matrix=pending_transform,
                        axis=axis_vector,
                        lower=coordinate.lower,
                        upper=coordinate.upper,
                        reference_axes=reference_axes,
                    )
                )
                robot.append(_make_dummy_link(child_link))
                current_parent = child_link
                pending_transform = _matrix_identity()
                variable_joint_count += 1
                continue

            if axis.coordinate and axis.function.kind in {"linear", "spline", "unsupported"}:
                warnings.append(
                    f"{joint.name}: freezing {axis.name} at the default value because URDF cannot represent its OpenSim coupling"
                )
            pending_transform = _matmul(
                pending_transform,
                _axis_fixed_transform(axis, joint.coordinates, warnings, joint.name),
            )

        child_transform = _transform_from_xyz_rpy(
            _rotate_vector(joint.location),
            _convert_orientation_rpy(joint.orientation),
        )
        final_transform = _matmul(pending_transform, _inverse_transform(child_transform))

        robot.append(
            _make_joint(
                name=f"{body.name}_fixed",
                joint_type="fixed",
                parent=current_parent,
                child=body.name,
                origin_matrix=final_transform,
            )
        )
        body_link = _make_body_link(body)
        for index, geometry in enumerate(body.display_geometries):
            mesh_path = _geometry_relative_path(osim_path, output_path, geometry.filename)
            geometry_path = osim_path.parent / "Geometry" / Path(geometry.filename).with_suffix(".obj")
            if not geometry_path.exists():
                warnings.append(f"{body.name}: missing display geometry {geometry_path}")
            body_link.append(_make_visual(body.name, geometry, index, mesh_path.as_posix()))
        robot.append(body_link)

    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return variable_joint_count, warnings


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert an OpenSim .osim model into a URDF file."
    )
    parser.add_argument("--osim", type=Path, required=True, help="Path to the OpenSim .osim file")
    parser.add_argument(
        "--output",
        type=Path,
        help="Output URDF path. Defaults to the input path with a .urdf suffix.",
    )
    parser.add_argument(
        "--reference-urdf",
        type=Path,
        default=DEFAULT_REFERENCE_URDF,
        help=(
            "Reference URDF used to override matching variable-joint axes "
            "(defaults to models/rajagopal_model/Rajagopal2015.urdf)."
        ),
    )
    args = parser.parse_args()

    output_path = args.output or args.osim.with_suffix(".urdf")
    joint_count, warnings = convert_osim_to_urdf(
        args.osim,
        output_path,
        reference_urdf=args.reference_urdf,
    )
    for warning in warnings:
        print(f"warning: {warning}", file=sys.stderr)
    print(
        f"Wrote {output_path} with {joint_count} variable joints derived from {args.osim}",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
