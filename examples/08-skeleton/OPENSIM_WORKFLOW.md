# OpenSim Tendon Path Workflow

This example now includes a small conversion utility for copying a muscle path
from an OpenSim `.osim` model into `muscles_fixed.xml`.

## What the script handles

- Static `PathPoint` locations.
- `ConditionalPathPoint` or `MovingPathPoint` entries only when a constant
  3D location is available in the XML.
- Body name remapping from OpenSim frame names to this example's URDF link
  names through `opensim_body_map.json`.

## What still needs manual review

- Wrapping surfaces in OpenSim are not recreated directly. If a muscle wraps,
  add intermediate waypoints by hand to approximate the wrapped path.
- Moving path points defined by coordinate functions often need a chosen pose
  before they can be turned into a single fixed waypoint.
- Frame conventions still need to be verified visually in the live viewer.

## Print one OpenSim muscle path

```bash
python3 examples/08-skeleton/convert_opensim_paths.py \
  --osim /path/to/model.osim \
  --muscle glut_max1_r \
  --body-map examples/08-skeleton/opensim_body_map.json \
  --print-only
```

## Patch one muscle in `muscles_fixed.xml`

```bash
python3 examples/08-skeleton/convert_opensim_paths.py \
  --osim /path/to/model.osim \
  --muscle glut_max1_r \
  --body-map examples/08-skeleton/opensim_body_map.json \
  --sai-muscles examples/08-skeleton/muscles_fixed.xml
```

## Generate a full `muscles_osim.xml`

Use the existing SAI file as a template, then replace every matched lower-body
muscle path that can be mapped from `gait2392_simbody.osim`.

```bash
python3 examples/08-skeleton/convert_opensim_paths.py \
  --osim examples/08-skeleton/gait2392_simbody.osim \
  --body-map examples/08-skeleton/opensim_body_map.json \
  --muscle-map examples/08-skeleton/opensim_muscle_map.json \
  --sai-muscles examples/08-skeleton/muscles_fixed.xml \
  --output examples/08-skeleton/muscles_osim.xml \
  --batch-from-template
```

## Proof-of-concept target

Start with `glut_max1_r` or another muscle that uses only pelvis and femur
points and does not wrap. Those are the easiest to match faithfully.

## Suggested workflow

1. Run the script with `--print-only` for one muscle.
2. Compare the printed waypoints against the existing block in
   `muscles_fixed.xml`.
3. Patch the SAI file.
4. With the skeleton viewer running, save the XML and confirm the tendon path
   refreshes in place.
5. If the OpenSim muscle uses wrapping, add a few extra intermediate waypoints
   to approximate the OpenSim path shape.
