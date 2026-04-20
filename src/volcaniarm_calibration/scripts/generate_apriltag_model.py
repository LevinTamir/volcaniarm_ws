#!/usr/bin/env python3
"""Generate a 3D-printable OpenSCAD model of an AprilTag.

White cells are raised above the black base for two-color printing.
M3 mounting holes are placed symmetrically around the tag center.
Base thickness is fixed at 4mm.

Usage:
  python3 generate_apriltag_model.py \\
    --tag-id 5 --total-size 78 --active-size 64 --hole-c2c 70
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def find_tag_image(tag_id: int) -> Path:
    """Find the tag PNG in the volcaniarm_description models directory."""
    # Script lives at src/volcaniarm_calibration/scripts/, so parent.parent
    # (= src/) is where we look for sibling packages.
    src_dir = Path(__file__).resolve().parent.parent.parent
    model_dir = src_dir / 'volcaniarm_description' / 'models' / f'Apriltag36_11_{tag_id:05d}'
    texture = model_dir / 'materials' / 'textures' / f'tag36_11_{tag_id:05d}.png'
    return texture


def _col_edges(border: float, active: float) -> list:
    """Cumulative x-positions for column boundaries in a 10-cell grid.
    Columns 0 and 9 are border-width; columns 1-8 are (active/8)-width."""
    cell = active / 8.0
    edges = [0.0, border]
    for i in range(1, 9):
        edges.append(border + i * cell)
    edges.append(border + active + border)
    return edges


def generate_scad(tag_image_path: str, active_size: float, border: float,
                  base_thickness: float, raised_height: float,
                  hole_radius: float, hole_margin: float,
                  fillet_radius: float) -> str:
    img = Image.open(tag_image_path).convert('L')
    cells = np.array(img.resize((10, 10), Image.NEAREST))
    is_white = cells > 128

    total_size = active_size + 2 * border
    total_h = base_thickness + raised_height + 1
    x_edges = _col_edges(border, active_size)
    y_edges = _col_edges(border, active_size)

    lines = []
    lines.append(f'// AprilTag 36h11 - 3D printable model')
    lines.append(f'// Total: {total_size}mm, active (black square): {active_size}mm, border: {border}mm')
    lines.append(f'// Base: {base_thickness}mm, raised: {raised_height}mm')
    lines.append(f'// M3 holes (r={hole_radius}mm) at {hole_margin}mm from corners')
    lines.append(f'$fn = 48;')
    lines.append(f'')
    lines.append(f'fillet_r = {fillet_radius};')
    lines.append(f'')
    lines.append(f'module rounded_rect(size, r) {{')
    lines.append(f'  offset(r=r) offset(delta=-r)')
    lines.append(f'    square([size[0], size[1]]);')
    lines.append(f'}}')
    lines.append(f'')
    lines.append(f'difference() {{')
    lines.append(f'  intersection() {{')
    lines.append(f'    // Clip everything to the rounded outline')
    lines.append(f'    linear_extrude(height={base_thickness + raised_height + 0.01})')
    lines.append(f'      rounded_rect([{total_size}, {total_size}], fillet_r);')
    lines.append(f'')
    lines.append(f'    union() {{')
    lines.append(f'      // Base plate')
    lines.append(f'      cube([{total_size}, {total_size}, {base_thickness}]);')
    lines.append(f'      // Raised white cells (as individual 3D cubes, each overlapping the base by 0.01mm)')

    raised_z = base_thickness - 0.01
    raised_h = raised_height + 0.02
    for row in range(10):
        col = 0
        while col < 10:
            if is_white[row][col]:
                run_start = col
                while col < 10 and is_white[row][col]:
                    col += 1
                x0 = x_edges[run_start]
                w = x_edges[col] - x_edges[run_start]
                y0 = y_edges[9 - row]
                h = y_edges[10 - row] - y_edges[9 - row]
                lines.append(f'      translate([{x0}, {y0}, {raised_z}]) cube([{w}, {h}, {raised_h}]);')
            else:
                col += 1

    lines.append(f'    }}')
    lines.append(f'  }}')
    lines.append(f'  // M3 mounting holes')
    for hx, hy in [(hole_margin, hole_margin),
                    (total_size - hole_margin, hole_margin),
                    (total_size - hole_margin, total_size - hole_margin),
                    (hole_margin, total_size - hole_margin)]:
        lines.append(f'  translate([{hx}, {hy}, -0.5]) cylinder(r={hole_radius}, h={total_h});')
    lines.append(f'}}')

    return '\n'.join(lines)


BASE_THICKNESS_MM = 4.0
RAISED_HEIGHT_MM = 1.0
FILLET_MM = 4.0
M3_HOLE_RADIUS_MM = 1.6  # 3.2mm diameter for M3 clearance


def main():
    parser = argparse.ArgumentParser(
        description='Generate a 3D-printable AprilTag OpenSCAD model.')
    parser.add_argument('--tag-id', type=int, required=True,
                        help='AprilTag ID (e.g., 20)')
    parser.add_argument('--total-size', type=float, required=True,
                        help='Total outer tag size in mm (e.g., 78)')
    parser.add_argument('--active-size', type=float, required=True,
                        help='Effective marker size - black square, in mm (e.g., 64)')
    parser.add_argument('--hole-c2c', type=float, required=True,
                        help='Mounting hole center-to-center distance in mm (e.g., 70)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output .scad file path (default: data/ in workspace)')
    args = parser.parse_args()

    if args.active_size >= args.total_size:
        print('Error: active-size must be less than total-size', file=sys.stderr)
        sys.exit(1)
    if args.hole_c2c >= args.total_size:
        print('Error: hole-c2c must be less than total-size', file=sys.stderr)
        sys.exit(1)

    border = (args.total_size - args.active_size) / 2.0
    hole_margin = (args.total_size - args.hole_c2c) / 2.0

    tag_image = str(find_tag_image(args.tag_id))
    if not Path(tag_image).exists():
        print(f'Error: Could not find tag image for ID {args.tag_id} at {tag_image}', file=sys.stderr)
        sys.exit(1)

    if args.output:
        output = args.output
    else:
        ws_root = Path(__file__).resolve().parent.parent.parent.parent
        data_dir = ws_root / 'data'
        data_dir.mkdir(exist_ok=True)
        output = str(data_dir / f'apriltag_36h11_id{args.tag_id:02d}_{int(round(args.total_size))}mm.scad')

    scad = generate_scad(tag_image, args.active_size, border,
                         BASE_THICKNESS_MM, RAISED_HEIGHT_MM,
                         M3_HOLE_RADIUS_MM, hole_margin, FILLET_MM)

    with open(output, 'w') as f:
        f.write(scad)

    print(f'OpenSCAD model saved: {output}')
    print(f'  Total: {args.total_size}mm, active: {args.active_size}mm, border: {border}mm')
    print(f'  Base: {BASE_THICKNESS_MM}mm, raised: {RAISED_HEIGHT_MM}mm, fillet: {FILLET_MM}mm')
    print(f'  M3 holes (r={M3_HOLE_RADIUS_MM}mm) at {hole_margin}mm from corners '
          f'({args.hole_c2c}mm center-to-center)')
    print(f'  To export STL: openscad -o output.stl {output}')


if __name__ == '__main__':
    main()
