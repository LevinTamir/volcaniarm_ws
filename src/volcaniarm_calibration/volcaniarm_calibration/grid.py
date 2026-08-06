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
                seed=(0.0, 0.0),
                joint_limit_min_rad: float = None):
    """Return (kept_goals, GridStats).

    Joint bounds: each elbow must satisfy
        joint_limit_min_rad + margin <= theta <= joint_limit_rad - margin
    with joint_limit_min_rad defaulting to the symmetric
    -joint_limit_rad when not given (the pre-asymmetric behaviour).

    Seeds chain: each IK call is seeded with the last valid solution,
    which keeps the branch continuous along the serpentine exactly the
    way the runner will traverse it. Rejected points do not advance
    the seed.
    """
    p = params or vk.Params()
    lim_max = joint_limit_rad - limit_margin_rad
    lim_min = (-joint_limit_rad if joint_limit_min_rad is None
               else joint_limit_min_rad) + limit_margin_rad
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
        if not (lim_min <= tl <= lim_max and lim_min <= tr <= lim_max):
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


# Generous search box for limits-derived recommendations; the arm's
# reachable lobe always fits inside it at this mechanism's scale.
_SEARCH_BOX = (-0.5, 0.5, 0.4, 1.0)
_SEARCH_SPACING = 0.025


def reachable_cloud(joint_limit_rad: float,
                    joint_limit_min_rad: float = None,
                    limit_margin_rad: float = 0.05,
                    closure_margin_m: float = 0.02,
                    spacing: float = _SEARCH_SPACING) -> list:
    """Kept (y, z) points of a coarse serpentine over the generous
    search box, filtered by the given joint range - the raw material
    for the dashboard's limits-derived recommendations."""
    y0, y1, z0, z1 = _SEARCH_BOX
    pts = serpentine(y0, y1, z0, z1, spacing)
    kept, _ = filter_grid(pts, joint_limit_rad, limit_margin_rad,
                          closure_margin_m,
                          joint_limit_min_rad=joint_limit_min_rad)
    return kept


def recommended_rectangle(joint_limit_rad: float,
                          joint_limit_min_rad: float = None,
                          spacing: float = _SEARCH_SPACING,
                          symmetric_y: bool = False,
                          height_m: float = None,
                          **filter_kwargs):
    """Recommended task rectangle fully inside the reachable set.

    With ``height_m`` set (the task-band mode the dashboard uses): the
    WIDEST rectangle of that fixed z-extent, ties broken toward the
    shallower placement — matching the P0-8a band-placement analysis
    (max-area instead tends to pick tall, narrow, deep rectangles,
    which is not what a weeding band wants). Without it: the classic
    maximal-area rectangle (histogram method).

    Returns (y0, y1, z0, z1) snapped to the search grid, or None when
    nothing fits. symmetric_y=True shrinks the result to |y0| == y1 —
    a subset, so still fully reachable; the arm is mirror-symmetric so
    asymmetry in the raw result is just grid snapping, not real reach.
    """
    kept = reachable_cloud(joint_limit_rad, joint_limit_min_rad,
                           spacing=spacing, **filter_kwargs)
    if not kept:
        return None
    key = lambda v: round(v, 6)  # noqa: E731
    ys = sorted({key(p[0]) for p in kept})
    zs = sorted({key(p[1]) for p in kept})
    yi = {v: i for i, v in enumerate(ys)}
    zi = {v: i for i, v in enumerate(zs)}
    occ = [[0] * len(ys) for _ in zs]
    for (y, z) in kept:
        occ[zi[key(z)]][yi[key(y)]] = 1

    best = None  # (score, i0, i1, j0, j1) in index space
    if height_m is not None:
        # Fixed-height band: slide a window of `rows` grid rows down the
        # occupancy grid; in each placement the widest contiguous run of
        # fully-occupied columns is the candidate. Score = width, then
        # prefer the shallower placement (smaller z of the bottom edge).
        rows = int(round(height_m / spacing)) + 1
        if rows > len(zs):
            rows = len(zs)
        for j0 in range(0, len(zs) - rows + 1):
            j1 = j0 + rows - 1
            run = 0
            for i in range(len(ys) + 1):
                full = (i < len(ys)
                        and all(occ[j][i] for j in range(j0, j1 + 1)))
                if full:
                    run += 1
                    continue
                if run > 0:
                    cand = (run, -zs[j1], i - run, i - 1, j0, j1)
                    if best is None or cand[:2] > best[:2]:
                        best = cand
                run = 0
        if best is None:
            return None
        _, _, i0, i1, j0, j1 = best
    else:
        heights = [0] * len(ys)
        for j, row in enumerate(occ):
            for i, v in enumerate(row):
                heights[i] = heights[i] + 1 if v else 0
            stack = []  # (leftmost column this height reaches, height)
            for i, h in enumerate(heights + [0]):
                start = i
                while stack and stack[-1][1] >= h:
                    s, sh = stack.pop()
                    area = sh * (i - s)
                    if best is None or area > best[0]:
                        best = (area, s, i - 1, j - sh + 1, j)
                    start = s
                stack.append((start, h))
        _, i0, i1, j0, j1 = best

    y0, y1, z0, z1 = ys[i0], ys[i1], zs[j0], zs[j1]
    if symmetric_y:
        half = min(-y0, y1)
        if half <= 0:
            return None
        # Snap to the grid so the bounds stay on kept points.
        half = round(half / spacing) * spacing
        y0, y1 = -half, half
    return (y0, y1, z0, z1)


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
                    help='measured mechanical limit, positive direction '
                         '[rad] (protocol 6b)')
    ap.add_argument('--joint-limit-min', type=float, default=None,
                    help='negative-direction limit [rad]; defaults to '
                         '-joint_limit (symmetric)')
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
        pts, args.joint_limit, args.limit_margin, args.closure_margin,
        joint_limit_min_rad=args.joint_limit_min)
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
                             args.closure_margin,
                             joint_limit_min_rad=args.joint_limit_min)[0]]
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
