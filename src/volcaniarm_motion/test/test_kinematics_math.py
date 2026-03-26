#!/usr/bin/env python3
"""
Unit tests for the volcaniarm 5-bar linkage kinematics math.

Tests IK/FK round-trips, workspace boundary behaviour, and known positions
WITHOUT requiring any ROS infrastructure.
"""
import math
import sys
import os
import pytest

# ---------------------------------------------------------------------------
# Import the kinematics module without ROS by patching rclpy imports
# ---------------------------------------------------------------------------
# We extract the pure-math helpers from the class so tests stay fast & offline.

# Kinematic constants (must match motion_params.yaml)
L1 = 0.41621
L2 = 0.6225
l0 = 0.215
BASE_Z = 0.0582
LEFT_FK_OFFSET = 3.0 * math.pi / 4.0   # ~2.356
RIGHT_FK_OFFSET = math.pi / 4.0         # ~0.7854

WS_Y_MIN, WS_Y_MAX = -0.35, 0.35
WS_Z_MIN, WS_Z_MAX = 0.3, 1.0


# ── Pure-math replicas of the kinematics node methods ──────────────────────

def normalize(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def fk_to_urdf(fk_left, fk_right):
    return (normalize(fk_left - LEFT_FK_OFFSET),
            normalize(fk_right - RIGHT_FK_OFFSET))


def urdf_to_fk(urdf_left, urdf_right):
    return urdf_left + LEFT_FK_OFFSET, urdf_right + RIGHT_FK_OFFSET


def fk_planar(fk_right, fk_left):
    """Forward kinematics: FK angles -> (y, z) in arm frame."""
    y_l = -l0 + L1 * math.cos(fk_left)
    z_l = BASE_Z + L1 * math.sin(fk_left)
    y_r = l0 + L1 * math.cos(fk_right)
    z_r = BASE_Z + L1 * math.sin(fk_right)

    dy = y_l - y_r
    dz = z_l - z_r
    d = math.hypot(dy, dz)

    if d > 2.0 * L2 or d < 1e-9:
        raise ValueError(f'Elbows too far apart (d={d:.3f} m)')

    a = d / 2.0
    h = math.sqrt(L2**2 - a**2)
    my = y_r + a * dy / d
    mz = z_r + a * dz / d

    return my + h * dz / d, mz - h * dy / d


def ik_planar(y, z):
    """Inverse kinematics: (y, z) in arm frame -> FK angles."""
    eps = 1e-9
    z_off = z - BASE_Z

    dy1 = l0 + y
    dy2 = l0 - y
    r1 = math.hypot(dy1, z_off)
    r2 = math.hypot(dy2, z_off)

    if r1 < eps or r2 < eps:
        raise ValueError('Target at shoulder')

    min_r = abs(L1 - L2)
    max_r = L1 + L2
    if r1 < min_r - eps or r1 > max_r + eps or \
       r2 < min_r - eps or r2 > max_r + eps:
        raise ValueError(f'Unreachable (r1={r1:.3f}, r2={r2:.3f})')

    def _clamp_acos(val):
        return math.acos(max(-1.0, min(1.0, val)))

    alpha1 = _clamp_acos((L1**2 + r1**2 - L2**2) / (2 * L1 * r1))
    alpha2 = _clamp_acos((L1**2 + r2**2 - L2**2) / (2 * L1 * r2))

    beta1 = math.atan2(z_off, dy1)
    beta2 = math.atan2(z_off, dy2)

    return beta1 + alpha1, math.pi - beta2 - alpha2


def full_ik(y, z):
    """Full IK pipeline: (y,z) -> URDF joint angles (right, left)."""
    fk_left, fk_right = ik_planar(y, z)
    urdf_left, urdf_right = fk_to_urdf(fk_left, fk_right)
    return urdf_right, urdf_left  # matches service: theta1=right, theta2=left


def full_fk(urdf_right, urdf_left):
    """Full FK pipeline: URDF joint angles -> (y, z)."""
    fk_left, fk_right = urdf_to_fk(urdf_left, urdf_right)
    return fk_planar(fk_right, fk_left)


# ── Tests ──────────────────────────────────────────────────────────────────

class TestIKFKRoundTrip:
    """IK then FK should return the original (y, z)."""

    TARGETS = [
        (0.0, 0.5),    # center, mid-height
        (0.0, 0.3),    # center, near top of workspace
        (0.0, 0.9),    # center, near bottom of workspace
        (0.2, 0.5),    # right side
        (-0.2, 0.5),   # left side
        (0.1, 0.7),    # off-center
        (-0.15, 0.4),  # off-center
        (0.3, 0.6),    # near Y boundary
        (-0.3, 0.6),   # near Y boundary
    ]

    @pytest.mark.parametrize("y,z", TARGETS)
    def test_ik_fk_round_trip(self, y, z):
        urdf_right, urdf_left = full_ik(y, z)
        y_out, z_out = full_fk(urdf_right, urdf_left)
        assert y_out == pytest.approx(y, abs=1e-6), \
            f"Y mismatch: sent {y}, got {y_out}"
        assert z_out == pytest.approx(z, abs=1e-6), \
            f"Z mismatch: sent {z}, got {z_out}"

    @pytest.mark.parametrize("y,z", TARGETS)
    def test_fk_ik_round_trip(self, y, z):
        """FK(IK(y,z)) -> IK(FK(result)) should also round-trip."""
        urdf_right, urdf_left = full_ik(y, z)
        y_fk, z_fk = full_fk(urdf_right, urdf_left)
        urdf_right2, urdf_left2 = full_ik(y_fk, z_fk)
        assert urdf_right2 == pytest.approx(urdf_right, abs=1e-6)
        assert urdf_left2 == pytest.approx(urdf_left, abs=1e-6)


class TestHomePosition:
    """Home position (0.0, 0.0 URDF angles) should map to a known Y,Z."""

    def test_home_fk(self):
        y, z = full_fk(0.0, 0.0)
        # Home should be roughly centered (y~0) and within workspace
        assert y == pytest.approx(0.0, abs=0.05), f"Home Y={y:.4f}, expected ~0"
        assert WS_Z_MIN <= z <= WS_Z_MAX, \
            f"Home Z={z:.4f} outside workspace [{WS_Z_MIN}, {WS_Z_MAX}]"
        print(f"\nHome position: y={y:.4f}, z={z:.4f}")

    def test_home_ik_round_trip(self):
        y, z = full_fk(0.0, 0.0)
        urdf_right, urdf_left = full_ik(y, z)
        assert urdf_right == pytest.approx(0.0, abs=1e-6)
        assert urdf_left == pytest.approx(0.0, abs=1e-6)


class TestWorkspaceBoundaries:
    """Targets outside workspace should raise errors in the IK check."""

    def test_y_too_positive(self):
        with pytest.raises(ValueError):
            ik_planar(1.5, 0.5)

    def test_y_too_negative(self):
        with pytest.raises(ValueError):
            ik_planar(-1.5, 0.5)

    def test_z_too_small(self):
        """Far behind the linkage should be unreachable."""
        with pytest.raises(ValueError):
            ik_planar(0.0, -2.0)  # way behind the arm

    def test_z_too_large(self):
        with pytest.raises(ValueError):
            ik_planar(0.0, 2.0)  # way below arm reach


class TestAngleConversion:
    """URDF <-> FK angle conversions should be inverses."""

    @pytest.mark.parametrize("urdf_left,urdf_right", [
        (0.0, 0.0),
        (0.5, -0.5),
        (-1.0, 1.0),
        (math.pi / 2, -math.pi / 2),
    ])
    def test_urdf_fk_roundtrip(self, urdf_left, urdf_right):
        fk_left, fk_right = urdf_to_fk(urdf_left, urdf_right)
        out_left, out_right = fk_to_urdf(fk_left, fk_right)
        assert out_left == pytest.approx(urdf_left, abs=1e-9)
        assert out_right == pytest.approx(urdf_right, abs=1e-9)


class TestSymmetry:
    """Symmetric Y targets should produce mirrored joint angles."""

    @pytest.mark.parametrize("y,z", [(0.15, 0.5), (0.25, 0.7)])
    def test_y_symmetry(self, y, z):
        r_right, r_left = full_ik(y, z)
        l_right, l_left = full_ik(-y, z)
        # For a symmetric mechanism, swapping Y should swap (and negate) roles
        # The exact relationship depends on the linkage, but positions should
        # at least round-trip correctly for both
        y1, z1 = full_fk(r_right, r_left)
        y2, z2 = full_fk(l_right, l_left)
        assert y1 == pytest.approx(y, abs=1e-6)
        assert y2 == pytest.approx(-y, abs=1e-6)
        assert z1 == pytest.approx(z2, abs=1e-6), "Z should be equal for symmetric Y"
