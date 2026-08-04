"""Sweep-grid generation for Experiment 0 (robot characterization).

Builds a serpentine (boustrophedon) list of (y, z) goals inside an
axis-aligned task rectangle and filters each point analytically with
the in-process kinematics (volcaniarm_kinematics_py), the same library
the CalibrationRunner uses for motion IK -- so a goal that passes here
cannot fail the runner's IK precheck for kinematic reasons.

Filter, per point (all must hold):
  1. inverse_ee returns valid=True (loop closes), seeded from the
     previous kept point for branch continuity along the serpentine.
  2. Both elbow angles within +-(joint_limit - limit_margin).
  3. closure_margin(theta_L, theta_R) > closure_margin_m: distance of
     the passive closure from the stretched (type-2) singularity where
     the distal links go collinear.

Joint limits are a parameter, not a constant: the URDF says +-3.14 and
the true mechanical limits are measured on hardware (protocol item 6b).

CLI preview (no ROS needed beyond the kinematics module):

    python3 -m volcaniarm_calibration.grid \
        --y0 -0.40 --y1 0.40 --z0 0.55 --z1 0.85 \
        --spacing 0.025 --joint-limit 1.13 --out goals.yaml

Prints kept/rejected counts by reason and a time estimate; --out
writes the flat [y0, z0, y1, z1, ...] list the accuracy_test node's
``goals`` parameter takes. --nine adds the 9 accuracy/repeatability
anchor points with ready-made per-point run commands.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass, field

import volcaniarm_kinematics_py as vk


@dataclass
class GridStats:
    kept: int = 0
    ik_invalid: int = 0
    joint_limit: int = 0
    closure_margin: int = 0
    rejected_points: list = field(default_factory=list)

    @property
    def total(self) -> int:
        return (self.kept + self.ik_invalid + self.joint_limit
                + self.closure_margin)


def serpentine(y0: float, y1: float, z0: float, z1: float,
               spacing: float) -> list:
    """Ordered (y, z) grid: rows along Y, stepping in Z, alternating
    Y direction each row so consecutive goals are one cell apart."""
    n_y = int(round((y1 - y0) / spacing)) + 1
    n_z = int(round((z1 - z0) / spacing)) + 1
    ys = [y0 + i * spacing for i in range(n_y)]
    pts = []
    for j in range(n_z):
        z = z0 + j * spacing
        row = ys if j % 2 == 0 else ys[::-1]
        pts.extend((y, z) for y in row)
    return pts


def filter_grid(points, joint_limit_rad: float,
                limit_margin_rad: float = 0.05,
                closure_margin_m: float = 0.02,
                params: 'vk.Params' = None,
                seed=(0.0, 0.0)):
    """Return (kept_goals, GridStats).

    Seeds chain: each IK call is seeded with the last valid solution,
    which keeps the branch continuous along the serpentine exactly the
    way the runner will traverse it. Rejected points do not advance
    the seed.
    """
    p = params or vk.Params()
    lim = joint_limit_rad - limit_margin_rad
    seed_l, seed_r = seed
    kept = []
    stats = GridStats()
    for (y, z) in points:
        ik = vk.inverse_ee(p, y, z, seed_l, seed_r)
        if not ik.valid:
            stats.ik_invalid += 1
            stats.rejected_points.append((y, z, 'ik'))
            continue
        tl, tr = float(ik.theta_left), float(ik.theta_right)
        if abs(tl) > lim or abs(tr) > lim:
            stats.joint_limit += 1
            stats.rejected_points.append((y, z, 'limit'))
            continue
        if vk.closure_margin(p, tl, tr) <= closure_margin_m:
            stats.closure_margin += 1
            stats.rejected_points.append((y, z, 'closure'))
            continue
        kept.append((y, z))
        seed_l, seed_r = tl, tr
        stats.kept += 1
    return kept, stats


def nine_points(y0: float, y1: float, z0: float, z1: float,
                inset: float = 0.025) -> list:
    """The 9 accuracy/repeatability anchor points of a task rectangle:
    4 corners + 4 edge midpoints + center, inset from the edges so the
    corner points sit near but not on the workspace boundary.

    Order: corners (--, +-, -+, ++), edge mids (bottom-Y, top-Y,
    left-Z, right-Z), center -- but each is a separate repeatability
    run anyway, so the order only names the runs.
    """
    ya, yb = y0 + inset, y1 - inset
    za, zb = z0 + inset, z1 - inset
    ym, zm = (ya + yb) / 2, (za + zb) / 2
    r = lambda v: round(v, 4)  # noqa: E731
    return [(r(ya), r(za)), (r(yb), r(za)), (r(ya), r(zb)), (r(yb), r(zb)),
            (r(ym), r(za)), (r(ym), r(zb)), (r(ya), r(zm)), (r(yb), r(zm)),
            (r(ym), r(zm))]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--y0', type=float, required=True)
    ap.add_argument('--y1', type=float, required=True)
    ap.add_argument('--z0', type=float, required=True)
    ap.add_argument('--z1', type=float, required=True)
    ap.add_argument('--spacing', type=float, default=0.025)
    ap.add_argument('--joint-limit', type=float, required=True,
                    help='measured mechanical limit [rad] (protocol 6b)')
    ap.add_argument('--limit-margin', type=float, default=0.05)
    ap.add_argument('--closure-margin', type=float, default=0.02)
    ap.add_argument('--sec-per-point', type=float, default=13.0)
    ap.add_argument('--out', type=str, default='',
                    help='write flat goals list as YAML for the node param')
    ap.add_argument('--nine', action='store_true',
                    help='also print the 9 accuracy/repeatability anchor '
                         'points (corners + edge mids + center, inset one '
                         'spacing) with per-point run commands')
    args = ap.parse_args(argv)

    pts = serpentine(args.y0, args.y1, args.z0, args.z1, args.spacing)
    kept, stats = filter_grid(
        pts, args.joint_limit, args.limit_margin, args.closure_margin)
    est_min = len(kept) * args.sec_per_point / 60.0
    print(f'grid {stats.total} pts -> kept {stats.kept} '
          f'(ik {stats.ik_invalid}, limit {stats.joint_limit}, '
          f'closure {stats.closure_margin} rejected)')
    print(f'estimated sweep: {est_min:.0f} min/pass '
          f'at {args.sec_per_point:.0f} s/point')
    if args.out:
        import yaml
        flat = [round(v, 4) for yz in kept for v in yz]
        with open(args.out, 'w') as f:
            yaml.safe_dump({'goals': flat}, f, default_flow_style=True)
        print(f'wrote {args.out}')
    if args.nine:
        anchors = nine_points(args.y0, args.y1, args.z0, args.z1,
                              inset=args.spacing)
        # Each anchor is its own single-target repeatability run seeded
        # from the home pose, so filter every point independently with
        # the default (0, 0) seed. Chaining seeds across the large
        # jumps between anchors wraps the IK branch and falsely
        # rejects reachable points.
        ok = [pt for pt in anchors
              if filter_grid([pt], args.joint_limit, args.limit_margin,
                             args.closure_margin)[0]]
        names = ['corner --', 'corner +-', 'corner -+', 'corner ++',
                 'mid bottom', 'mid top', 'mid left', 'mid right',
                 'center']
        print('\n9 anchor points (repeatability test, 30 cycles each '
              '-> accuracy AP + repeatability RP per ISO 9283):')
        for name, (y, z) in zip(names, anchors):
            mark = 'OK  ' if (y, z) in ok else 'FAIL'
            print(f'  {mark} {name:10s} ({y:+.3f}, {z:.3f})  '
                  f'-p test_type:=repeatability -p num_cycles:=30 '
                  f'-p goals:="[{y:.3f}, {z:.3f}]"')
        if len(ok) < 9:
            print('  FAIL points are outside the filtered workspace -- '
                  'increase the inset or shrink the rectangle.')


if __name__ == '__main__':
    main()
