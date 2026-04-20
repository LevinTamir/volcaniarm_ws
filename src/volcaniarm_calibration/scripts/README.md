# AprilTag CAD Generator

`generate_apriltag_model.py` produces a 3D-printable OpenSCAD model of an AprilTag 36h11 marker with raised white cells and M3 mounting holes.

## Prerequisites

The tag texture PNG must exist at:
```
volcaniarm_description/models/Apriltag36_11_{ID:05d}/materials/textures/tag36_11_{ID:05d}.png
```

Textures for IDs 0-15 can be copied from [gazebo_apriltag](https://github.com/koide3/gazebo_apriltag) into `volcaniarm_description/models/`.

## Usage

```bash
cd /home/tamir/workspaces/volcaniarm_ws

# Generate SCAD
python3 src/volcaniarm_calibration/scripts/generate_apriltag_model.py \
  --tag-id 5 \
  --total-size 78 \
  --active-size 64 \
  --hole-c2c 70

# Export STL
openscad -o data/apriltag_36h11_id05_78mm.stl data/apriltag_36h11_id05_78mm.scad
```

## Required arguments

| Arg | Description | Example |
|---|---|---|
| `--tag-id` | AprilTag ID | `5` |
| `--total-size` | Full outer tag size in mm | `78` |
| `--active-size` | Black-square size in mm (this is `size` in `apriltag_params.yaml`) | `64` |
| `--hole-c2c` | M3 hole center-to-center distance in mm | `70` |

Optional: `--output PATH` to override the default `data/apriltag_36h11_id{ID}_{TOTAL}mm.scad`.

## Fixed defaults

- Base thickness: 4 mm
- Raised pattern height: 1 mm
- M3 hole radius: 1.6 mm (3.2 mm diameter, M3 clearance)
- Corner fillet: 4 mm

Edit the constants at the top of `generate_apriltag_model.py` to change these.

## Size reference

AprilTag 36h11 layout is 10x10 cells total: 8x8 inner (the "active" / detection square) + 1 white border cell on each side. The `apriltag_ros` `size` parameter refers to the **active size** (black square), not the total outer size.

| Printed total | Active (black square) | White border |
|---|---|---|
| 78 mm | 64 mm | 7 mm |
| 200 mm | 160 mm | 20 mm |
