#!/usr/bin/env python3
"""Generate a 3D-printable OpenSCAD model of an AprilTag.

White cells are raised above the black base for two-color printing.
M3 mounting holes are placed in the corners.

Usage:
  python3 generate_apriltag_model.py [--output PATH] [--size 200] [--base 4] [--raised 1]
"""

import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image


def find_tag_image(tag_id: int) -> Path:
    """Find the tag PNG in the volcaniarm_description models directory."""
    script_dir = Path(__file__).resolve().parent
    ws_root = script_dir.parent.parent.parent
    model_dir = ws_root / 'volcaniarm_description' / 'models' / f'Apriltag36_11_{tag_id:05d}'
    texture = model_dir / 'materials' / 'textures' / f'tag36_11_{tag_id:05d}.png'
    if not texture.exists():
        # Try installed path
        texture = model_dir / 'materials' / 'textures' / f'tag36_11_{tag_id:05d}.png'
    return texture


def generate_scad(tag_image_path: str, tag_size: float, base_thickness: float,
                  raised_height: float, hole_radius: float, hole_margin: float,
                  fillet_radius: float) -> str:
    img = Image.open(tag_image_path).convert('L')
    cells = np.array(img.resize((10, 10), Image.NEAREST))
    is_white = cells > 128

    cell_size = tag_size / 10.0
    total_h = base_thickness + raised_height + 1

    lines = []
    lines.append(f'// AprilTag 36h11 - 3D printable model')
    lines.append(f'// Tag size: {tag_size}mm, base: {base_thickness}mm, raised: {raised_height}mm')
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
    lines.append(f'    linear_extrude(height={base_thickness + raised_height + 0.01})')
    lines.append(f'      rounded_rect([{tag_size}, {tag_size}], fillet_r);')
    lines.append(f'    union() {{')
    lines.append(f'      cube([{tag_size}, {tag_size}, {base_thickness}]);')
    lines.append(f'      translate([0, 0, {base_thickness - 0.01}])')
    lines.append(f'        linear_extrude(height={raised_height + 0.01})')
    lines.append(f'          union() {{')

    for row in range(10):
        col = 0
        while col < 10:
            if is_white[row][col]:
                run_start = col
                while col < 10 and is_white[row][col]:
                    col += 1
                run_len = col - run_start
                x0 = run_start * cell_size
                y0 = (9 - row) * cell_size
                w = run_len * cell_size
                lines.append(f'            translate([{x0}, {y0}]) square([{w}, {cell_size}]);')
            else:
                col += 1

    lines.append(f'          }}')
    lines.append(f'    }}')
    lines.append(f'  }}')
    lines.append(f'  // M3 mounting holes')
    for hx, hy in [(hole_margin, hole_margin),
                    (tag_size - hole_margin, hole_margin),
                    (tag_size - hole_margin, tag_size - hole_margin),
                    (hole_margin, tag_size - hole_margin)]:
        lines.append(f'  translate([{hx}, {hy}, -0.5]) cylinder(r={hole_radius}, h={total_h});')
    lines.append(f'}}')

    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description='Generate 3D-printable AprilTag OpenSCAD model')
    parser.add_argument('--tag-id', type=int, default=20, help='AprilTag ID (default: 20)')
    parser.add_argument('--tag-image', type=str, default=None,
                        help='Path to tag PNG (auto-detected if omitted)')
    parser.add_argument('--output', '-o', type=str, default=None,
                        help='Output .scad file path (default: data/ in workspace)')
    parser.add_argument('--size', type=float, default=200.0, help='Tag size in mm (default: 200)')
    parser.add_argument('--base', type=float, default=4.0, help='Base thickness in mm (default: 4)')
    parser.add_argument('--raised', type=float, default=1.0, help='Raised height in mm (default: 1)')
    parser.add_argument('--hole-radius', type=float, default=1.6,
                        help='Mounting hole radius in mm (default: 1.6 for M3)')
    parser.add_argument('--hole-margin', type=float, default=5.0,
                        help='Hole distance from corner in mm (default: 5)')
    parser.add_argument('--fillet', type=float, default=4.0,
                        help='Corner fillet radius in mm (default: 4)')
    args = parser.parse_args()

    if args.tag_image:
        tag_image = args.tag_image
    else:
        tag_image = str(find_tag_image(args.tag_id))
        if not Path(tag_image).exists():
            print(f'Error: Could not find tag image for ID {args.tag_id}', file=sys.stderr)
            sys.exit(1)

    if args.output:
        output = args.output
    else:
        ws_root = Path(__file__).resolve().parent.parent.parent.parent
        data_dir = ws_root / 'data'
        data_dir.mkdir(exist_ok=True)
        output = str(data_dir / f'apriltag_36h11_id{args.tag_id:02d}_{int(args.size)}mm.scad')

    scad = generate_scad(tag_image, args.size, args.base, args.raised,
                         args.hole_radius, args.hole_margin, args.fillet)

    with open(output, 'w') as f:
        f.write(scad)

    print(f'OpenSCAD model saved: {output}')
    print(f'  Tag size: {args.size}mm, base: {args.base}mm, raised: {args.raised}mm')
    print(f'  Fillet: {args.fillet}mm, M3 holes at {args.hole_margin}mm from corners')
    print(f'  To export STL: openscad -o output.stl {output}')


if __name__ == '__main__':
    main()
