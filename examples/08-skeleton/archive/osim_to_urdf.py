
import xml.etree.ElementTree as ET
import math
import re
from pathlib import Path


def vec3(text, default=(0.0, 0.0, 0.0)):
    if text is None or not str(text).strip():
        return default
    vals = [float(x) for x in str(text).strip().split()]
    return tuple(vals) if len(vals) == 3 else default


def sanitize(s):
    return re.sub(r'[^A-Za-z0-9_]+', '_', s)


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        [cp * cy, -cp * sy, sp],
        [sr * sp * cy + cr * sy, -sr * sp * sy + cr * cy, -sr * cp],
        [-cr * sp * cy + sr * sy, cr * sp * sy + sr * cy, cr * cp],
    ]


def matmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def matT(A):
    return [[A[j][i] for j in range(3)] for i in range(3)]


def compose(T1, T2):
    p1, R1 = T1
    p2, R2 = T2
    p = [p1[i] + sum(R1[i][k] * p2[k] for k in range(3)) for i in range(3)]
    R = matmul(R1, R2)
    return p, R


def invT(T):
    p, R = T
    Rt = matT(R)
    pi = [-sum(Rt[i][k] * p[k] for k in range(3)) for i in range(3)]
    return pi, Rt


def matrix_to_rpy(R):
    sp = max(-1.0, min(1.0, R[0][2]))
    p = math.asin(sp)
    cp = math.cos(p)
    if abs(cp) < 1e-8:
        y = 0.0
        r = math.atan2(-R[1][0], R[1][1])
    else:
        y = math.atan2(-R[0][1], R[0][0])
        r = math.atan2(-R[1][2], R[2][2])
    return (r, p, y)


def fmt(vals):
    return " ".join(f"{v:.8g}" for v in vals)


def parse_coordinate(coord_elem):
    return {
        'name': sanitize(coord_elem.attrib['name']),
        'orig_name': coord_elem.attrib['name'],
        'motion_type': (coord_elem.findtext('motion_type') or 'rotational').strip().lower(),
        'default': float((coord_elem.findtext('default_value') or '0').strip()),
        'range': tuple(float(x) for x in (coord_elem.findtext('range') or '0 0').split()[:2]),
        'clamped': ((coord_elem.findtext('clamped') or 'false').strip().lower() == 'true'),
        'locked': ((coord_elem.findtext('locked') or 'false').strip().lower() == 'true'),
    }


def parse_joint(j):
    data = {
        'type': j.tag,
        'name': sanitize(j.attrib['name']),
        'orig_name': j.attrib['name'],
        'parent': sanitize((j.findtext('parent_body') or '').strip()),
        'parent_loc': vec3(j.findtext('location_in_parent') or '0 0 0'),
        'parent_rpy': vec3(j.findtext('orientation_in_parent') or '0 0 0'),
        'child_loc': vec3(j.findtext('location') or '0 0 0'),
        'child_rpy': vec3(j.findtext('orientation') or '0 0 0'),
        'coords': [],
        'spatial_axes': [],
    }
    coords = j.find('CoordinateSet/objects')
    if coords is not None:
        data['coords'] = [parse_coordinate(c) for c in coords]
    st = j.find('SpatialTransform')
    if st is not None:
        for ta in list(st):
            axis = {
                'coord': sanitize((ta.findtext('coordinates') or '').strip()),
                'axis': vec3(ta.findtext('axis') or '0 0 1'),
                'function_type': None,
            }
            for desc in ta.iter():
                if desc is ta:
                    continue
                if desc.tag in ('LinearFunction', 'Constant', 'SimmSpline', 'MultiplierFunction'):
                    axis['function_type'] = desc.tag
                    if desc.tag == 'LinearFunction':
                        axis['coefficients'] = (desc.findtext('coefficients') or '').strip()
                    elif desc.tag == 'Constant':
                        axis['value'] = (desc.findtext('value') or '').strip()
                    elif desc.tag == 'SimmSpline':
                        axis['x'] = (desc.findtext('x') or '').strip()
                        axis['y'] = (desc.findtext('y') or '').strip()
                    break
            data['spatial_axes'].append(axis)
    return data


def body_data(body_elem):
    name = sanitize(body_elem.attrib['name'])

    def ftext(tag, default='0'):
        e = body_elem.find(tag)
        return e.text.strip() if e is not None and e.text else default

    inert = {
        'mass': float(ftext('mass', '0')),
        'com': vec3(ftext('mass_center', '0 0 0')),
        'ixx': float(ftext('inertia_xx', '0')),
        'iyy': float(ftext('inertia_yy', '0')),
        'izz': float(ftext('inertia_zz', '0')),
        'ixy': float(ftext('inertia_xy', '0')),
        'ixz': float(ftext('inertia_xz', '0')),
        'iyz': float(ftext('inertia_yz', '0')),
    }
    joint_container = body_elem.find('Joint')
    joint = parse_joint(list(joint_container)[0]) if joint_container is not None and len(joint_container) else None
    return {'name': name, 'inertial': inert, 'joint': joint}


def joint_origin_from_opensim(j):
    Tp = (j['parent_loc'], rpy_to_matrix(j['parent_rpy']))
    Tc = (j['child_loc'], rpy_to_matrix(j['child_rpy']))
    T = compose(Tp, invT(Tc))
    return T[0], matrix_to_rpy(T[1])


def make_joint_block(name, jtype, parent, child, xyz, rpy, axis=None, coord=None, comment=None):
    lines = []
    if comment:
        lines.append(f'  <!-- {comment} -->')
    lines.append(f'  <joint name="{name}" type="{jtype}">')
    lines.append(f'    <parent link="{parent}"/>')
    lines.append(f'    <child link="{child}"/>')
    lines.append(f'    <origin xyz="{fmt(xyz)}" rpy="{fmt(rpy)}"/>')
    if axis is not None and jtype not in ('fixed', 'floating'):
        lines.append(f'    <axis xyz="{fmt(axis)}"/>')
    if coord is not None and jtype not in ('fixed', 'floating'):
        lo, hi = coord["range"]
        lines.append(f'    <limit lower="{lo:.8g}" upper="{hi:.8g}" effort="1000" velocity="10"/>')
    lines.append('  </joint>')
    return "\n".join(lines)


def convert(osim_path, urdf_path):
    root = ET.parse(osim_path).getroot()
    model = root.find('Model')
    bodies = [body_data(b) for b in model.findall('.//BodySet/objects/Body')]

    links_xml = []
    joints_xml = []

    for b in bodies:
        links_xml.append(f'  <link name="{b["name"]}">')
        inert = b['inertial']
        links_xml.append('    <inertial>')
        links_xml.append(f'      <origin xyz="{fmt(inert["com"])}" rpy="0 0 0"/>')
        links_xml.append(f'      <mass value="{inert["mass"]:.8g}"/>')
        links_xml.append(
            f'      <inertia ixx="{inert["ixx"]:.8g}" ixy="{inert["ixy"]:.8g}" ixz="{inert["ixz"]:.8g}" '
            f'iyy="{inert["iyy"]:.8g}" iyz="{inert["iyz"]:.8g}" izz="{inert["izz"]:.8g}"/>'
        )
        links_xml.append('    </inertial>')
        links_xml.append('  </link>')

    for b in bodies:
        if b['name'] == 'ground' or b['joint'] is None:
            continue

        j = b['joint']
        xyz0, rpy0 = joint_origin_from_opensim(j)
        parent = j['parent']
        child = b['name']
        coords_by_name = {c['name']: c for c in j['coords']}

        if j['type'] == 'PinJoint':
            coord = j['coords'][0] if j['coords'] else None
            joints_xml.append(
                make_joint_block(
                    j['name'], 'revolute', parent, child, xyz0, rpy0, axis=(0, 0, 1), coord=coord,
                    comment=f"Converted from OpenSim PinJoint '{j['orig_name']}' (rotation about joint Z axis)."
                )
            )

        elif j['type'] == 'UniversalJoint':
            inter = sanitize(j['name'] + '_u0')
            links_xml.append(f'  <link name="{inter}"/>')
            c0 = j['coords'][0] if len(j['coords']) > 0 else None
            c1 = j['coords'][1] if len(j['coords']) > 1 else None
            joints_xml.append(
                make_joint_block(
                    j['name'] + '_' + (c0['name'] if c0 else 'x'),
                    'revolute', parent, inter, xyz0, rpy0, axis=(1, 0, 0), coord=c0,
                    comment=f"Approximation of OpenSim UniversalJoint '{j['orig_name']}' as two serial revolute joints."
                )
            )
            joints_xml.append(
                make_joint_block(
                    j['name'] + '_' + (c1['name'] if c1 else 'y'),
                    'revolute', inter, child, (0, 0, 0), (0, 0, 0), axis=(0, 1, 0), coord=c1
                )
            )

        elif j['type'] == 'CustomJoint':
            usable = []
            coupled = False
            for ax in j['spatial_axes']:
                cname = ax['coord']
                if not cname or cname not in coords_by_name:
                    continue
                coord = coords_by_name[cname]
                axis_adj = None
                if ax.get('function_type') == 'LinearFunction':
                    coeffs = ax.get('coefficients', '').split()
                    if len(coeffs) >= 1:
                        a = float(coeffs[0])
                        b = float(coeffs[1]) if len(coeffs) > 1 else 0.0
                        if abs(b) < 1e-9 and abs(abs(a) - 1.0) < 1e-9:
                            s = 1.0 if a > 0 else -1.0
                            axis_adj = tuple(s * v for v in ax['axis'])
                if axis_adj is not None:
                    jt = 'prismatic' if coord['motion_type'].startswith('trans') else 'revolute'
                    usable.append((cname, jt, axis_adj, coord))
                else:
                    coupled = True

            ordered = []
            seen = set()
            for cname, jt, axis, coord in usable:
                if cname not in seen:
                    seen.add(cname)
                    ordered.append((cname, jt, axis, coord))
            usable = ordered

            if len(usable) == 0:
                joints_xml.append(
                    make_joint_block(
                        j['name'], 'fixed', parent, child, xyz0, rpy0,
                        comment=f"OpenSim CustomJoint '{j['orig_name']}' collapsed to fixed joint; coupled transform not representable in URDF."
                    )
                )
            elif len(usable) == 1 and coupled:
                cname, jt, axis, coord = usable[0]
                joints_xml.append(
                    make_joint_block(
                        coord['name'], jt, parent, child, xyz0, rpy0, axis=axis, coord=coord,
                        comment=f"Approximation of OpenSim CustomJoint '{j['orig_name']}'. Kept primary DOF '{coord['orig_name']}', ignored coupled translations/rotations."
                    )
                )
            else:
                prev_parent = parent
                first = True
                for idx, (cname, jt, axis, coord) in enumerate(usable):
                    last = idx == (len(usable) - 1)
                    link_name = child if last else sanitize(f"{j['name']}_frame_{idx}")
                    if not last:
                        links_xml.append(f'  <link name="{link_name}"/>')
                    xyz, rpy = (xyz0, rpy0) if first else ((0, 0, 0), (0, 0, 0))
                    comment = None
                    if first:
                        comment = (
                            f"Approximation of OpenSim CustomJoint '{j['orig_name']}' as a URDF serial chain. "
                            f"Coupled non-identity transform axes were ignored."
                            if coupled else
                            f"Converted from OpenSim CustomJoint '{j['orig_name']}' as a URDF serial chain."
                        )
                    joints_xml.append(make_joint_block(coord['name'], jt, prev_parent, link_name, xyz, rpy, axis=axis, coord=coord, comment=comment))
                    prev_parent = link_name
                    first = False

        else:
            joints_xml.append(
                make_joint_block(
                    j['name'], 'fixed', parent, child, xyz0, rpy0,
                    comment=f"Unsupported OpenSim joint type '{j['type']}' converted to fixed."
                )
            )

    urdf = [
        '<?xml version="1.0"?>',
        '<robot name="Rajagopal2015_skeleton">',
        '  <!-- Generated from Rajagopal2015.osim as a skeleton-only URDF. -->',
        '  <!-- Muscles, wrapping objects, constraints, and visual meshes were omitted. -->',
        '  <!-- OpenSim CustomJoint spline/coupled transforms were approximated where necessary. -->',
    ]
    urdf += links_xml + joints_xml + ['</robot>']

    Path(urdf_path).write_text("\n".join(urdf))


if __name__ == "__main__":
    import sys
    if len(sys.argv) != 3:
        raise SystemExit("Usage: python osim_to_urdf.py input.osim output.urdf")
    convert(sys.argv[1], sys.argv[2])
