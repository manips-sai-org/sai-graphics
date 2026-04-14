#!/usr/bin/env python3

import argparse
from pathlib import Path

import vtk


def convert_vtp_to_obj(input_path: Path, output_path: Path) -> None:
    reader = vtk.vtkXMLPolyDataReader()
    reader.SetFileName(str(input_path))
    reader.Update()

    poly_data = reader.GetOutput()
    if poly_data is None or poly_data.GetNumberOfPoints() == 0:
        raise ValueError(f"No polydata loaded from {input_path}")

    triangle_filter = vtk.vtkTriangleFilter()
    triangle_filter.SetInputData(poly_data)
    triangle_filter.Update()

    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(triangle_filter.GetOutputPort())
    normals.SplittingOff()
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()
    normals.Update()

    output_path.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkOBJWriter()
    writer.SetFileName(str(output_path))
    writer.SetInputConnection(normals.GetOutputPort())
    writer.Write()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Convert VTP meshes to OBJ."
    )
    parser.add_argument(
        "inputs",
        nargs="+",
        type=Path,
        help="Input .vtp files or directories containing .vtp files.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="Optional output directory. Defaults to each input file's directory.",
    )
    args = parser.parse_args()

    input_files = []
    for input_path in args.inputs:
        if input_path.is_dir():
            input_files.extend(sorted(input_path.glob("*.vtp")))
        else:
            input_files.append(input_path)

    if not input_files:
        raise ValueError("No input .vtp files found.")

    for input_path in input_files:
        if input_path.suffix.lower() != ".vtp":
            raise ValueError(f"Expected a .vtp file, got {input_path}")
        if args.output_dir is None:
            output_path = input_path.with_suffix(".obj")
        else:
            output_path = args.output_dir / input_path.with_suffix(".obj").name
        convert_vtp_to_obj(input_path, output_path)
        print(f"wrote {output_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
