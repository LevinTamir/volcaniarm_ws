#!/usr/bin/env python3
"""
Compare the planar FK model against the actual URDF kinematic chain.

The FK model in volcaniarm_kinematics.py uses simplified 2D geometry with
parameters (L1, L2, l0, base_z, fk_offsets).  This test computes the arm
tip position by walking the full URDF joint/link chain in 3D and checks
whether the FK model agrees.

If there is a mismatch, the arm in Gazebo goes to a different position than
what the FK model predicts — which is exactly the "not reaching the plant" bug.

URDF chain (right side, from volcaniarm_base_link):
  volcaniarm_base_link
    -> volcaniarm_right_elbow_link   revolute x-axis, origin xyz=(0.289, 0.215, 0.0632) rpy=(-0.7854, 0, 0)
       -> volcaniarm_right_arm_link  revolute x-axis, origin xyz=(0.004, -0.02, 0.41621) rpy=(1.7053, 0, 0)
          -> right_arm_tip_link      fixed,           origin xyz=(0, 0, 0.6225) rpy=(0, 0, 0)

Left side:
  volcaniarm_base_link
    -> volcaniarm_left_elbow_link    revolute x-axis, origin xyz=(0.285, -0.215, 0.0632) rpy=(0.7854, 0, 0)
       -> volcaniarm_left_arm_link   revolute x-axis, origin xyz=(0.004, 0.02, 0.41621) rpy=(-1.7053, 0, 0)
          -> closure_dummy_link      revolute x-axis, origin xyz=(0, 0, 0.6225) rpy=(1.8398, 0, 0)
"""
import math
import numpy as np
import pytest

from test_kinematics_math import (
    full_fk, full_ik,
    L1, L2, l0, BASE_Z, LEFT_FK_OFFSET, RIGHT_FK_OFFSET,
    urdf_to_fk, fk_to_urdf, fk_planar,
)


# ---------------------------------------------------------------------------
# URDF forward kinematics (full 3D, using homogeneous transforms)
# ---------------------------------------------------------------------------

def _tf(xyz, rpy):
    """4x4 homogeneous transform from xyz + rpy."""
    x, y, z = xyz
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    R = np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [  -sp,            cp*sr,            cp*cr],
    ])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def _rot_x(angle):
    """4x4 rotation about X axis."""
    c, s = math.cos(angle), math.sin(angle)
    T = np.eye(4)
    T[1, 1] = c; T[1, 2] = -s
    T[2, 1] = s; T[2, 2] = c
    return T


def urdf_right_tip(urdf_right_angle):
    """
    Compute right_arm_tip_link position in volcaniarm_base_link frame
    using the full URDF 3D chain.
    """
    # Joint: volcaniarm_right_elbow_joint
    # origin xyz=(0.289, 0.215, 0.0632) rpy=(-0.7854, 0, 0), axis=X
    T = _tf([0.289, 0.215, 0.0632], [-0.7854, 0, 0])
    T = T @ _rot_x(urdf_right_angle)

    # Joint: volcaniarm_right_arm_joint (passive)
    # origin xyz=(0.004, -0.02, 0.41621) rpy=(1.7053, 0, 0), axis=X
    # This is a passive joint — in the closed-loop, its angle is determined
    # by the constraint.  For now, we set it to 0 to see the URDF geometry.
    T_arm_joint = _tf([0.004, -0.02, 0.41621], [1.7053, 0, 0])

    # right_arm_tip_link: fixed joint
    # origin xyz=(0, 0, 0.6225) rpy=(0, 0, 0)
    T_tip = _tf([0, 0, 0.6225], [0, 0, 0])

    return T, T_arm_joint, T_tip


def urdf_left_tip(urdf_left_angle):
    """
    Compute left arm endpoint in volcaniarm_base_link frame
    using the full URDF 3D chain.
    """
    # Joint: volcaniarm_left_elbow_joint
    # origin xyz=(0.285, -0.215, 0.0632) rpy=(0.7854, 0, 0), axis=X
    T = _tf([0.285, -0.215, 0.0632], [0.7854, 0, 0])
    T = T @ _rot_x(urdf_left_angle)

    # Joint: volcaniarm_left_arm_joint (passive)
    # origin xyz=(0.004, 0.02, 0.41621) rpy=(-1.7053, 0, 0), axis=X
    T_arm_joint = _tf([0.004, 0.02, 0.41621], [-1.7053, 0, 0])

    # closure_dummy_link: origin xyz=(0, 0, 0.6225)
    T_tip = _tf([0, 0, 0.6225], [1.8398, 0, 0])

    return T, T_arm_joint, T_tip


def urdf_elbow_position(side, urdf_angle):
    """
    Compute the elbow endpoint (where the arm link starts) in base_link frame.
    This is the point at the end of the L1 (elbow) link — before the arm joint.
    """
    if side == 'right':
        T = _tf([0.289, 0.215, 0.0632], [-0.7854, 0, 0])
        T = T @ _rot_x(urdf_angle)
        # The arm joint origin gives the elbow endpoint
        elbow_end_local = np.array([0.004, -0.02, 0.41621, 1.0])
    else:
        T = _tf([0.285, -0.215, 0.0632], [0.7854, 0, 0])
        T = T @ _rot_x(urdf_angle)
        elbow_end_local = np.array([0.004, 0.02, 0.41621, 1.0])

    elbow_end = T @ elbow_end_local
    return elbow_end[:3]


# ---------------------------------------------------------------------------
# Tests: Compare FK model parameters against URDF geometry
# ---------------------------------------------------------------------------

class TestFKParametersMatchURDF:
    """
    Check that the simplified FK model's parameters (L1, L2, l0, base_z)
    match the actual URDF geometry.
    """

    def test_shoulder_separation_l0(self):
        """FK model l0=0.215 should match URDF shoulder Y offsets."""
        # Right elbow joint Y = +0.215
        # Left elbow joint Y = -0.215
        # FK model: right shoulder at +l0, left at -l0
        right_y = 0.215  # from URDF
        left_y = -0.215  # from URDF
        assert l0 == pytest.approx(right_y, abs=1e-4), \
            f"FK l0={l0} != URDF right shoulder Y={right_y}"
        assert l0 == pytest.approx(-left_y, abs=1e-4), \
            f"FK l0={l0} != URDF |left shoulder Y|={-left_y}"
        print(f"\nl0: FK={l0}, URDF right_Y={right_y}, left_Y={left_y} -- MATCH")

    def test_base_z_offset(self):
        """FK model base_z=0.0582 should match URDF shoulder Z offset."""
        # Both elbow joints have Z=0.0632 in URDF
        urdf_z = 0.0632
        print(f"\nbase_z: FK={BASE_Z}, URDF={urdf_z}, diff={BASE_Z - urdf_z:.4f}")
        # NOTE: These might not match exactly — the FK model may account for
        # the rpy rotation shifting the effective Z
        if abs(BASE_Z - urdf_z) > 0.01:
            print("  WARNING: base_z differs by more than 1cm!")

    def test_elbow_link_length_L1(self):
        """FK model L1=0.41621 should match URDF elbow-to-arm-joint distance."""
        # volcaniarm_right_arm_joint origin: xyz=(0.004, -0.02, 0.41621)
        # The Z component is 0.41621 — this is L1
        urdf_L1_z = 0.41621
        urdf_L1_actual = math.sqrt(0.004**2 + 0.02**2 + 0.41621**2)
        print(f"\nL1: FK={L1}, URDF z-component={urdf_L1_z}, "
              f"URDF 3D distance={urdf_L1_actual:.5f}")
        assert L1 == pytest.approx(urdf_L1_z, abs=1e-4), \
            f"FK L1={L1} != URDF elbow link Z={urdf_L1_z}"

    def test_arm_link_length_L2(self):
        """FK model L2=0.65 should match URDF arm-to-tip distance."""
        # right_arm_tip_joint origin: xyz=(0, 0, 0.6225)
        # But wait — L2=0.65 != 0.6225!
        urdf_tip_z = 0.6225
        print(f"\nL2: FK={L2}, URDF tip z-component={urdf_tip_z}, "
              f"diff={L2 - urdf_tip_z:.4f}")
        if abs(L2 - urdf_tip_z) > 0.01:
            print(f"  *** MISMATCH: FK uses L2={L2} but URDF arm length is {urdf_tip_z} ***")
            print(f"  *** This means FK predicts the tip {L2 - urdf_tip_z:.4f}m further "
                  f"than the URDF! ***")

    def test_shoulder_x_offsets(self):
        """Check if the FK model accounts for X offsets in the shoulder joints."""
        # Right: X=0.289, Left: X=0.285
        # FK model is purely in the YZ plane — it ignores X entirely
        right_x = 0.289
        left_x = 0.285
        print(f"\nShoulder X offsets: right={right_x}, left={left_x}")
        print(f"  FK model ignores X (2D planar). Asymmetry={right_x - left_x:.4f}m")
        if abs(right_x - left_x) > 0.001:
            print("  NOTE: Left/right shoulders are NOT symmetric in X")

    def test_arm_joint_offsets(self):
        """Check the small XY offsets in the arm joints."""
        # Right arm joint: xyz=(0.004, -0.02, 0.41621)
        # Left arm joint:  xyz=(0.004, 0.02, 0.41621)
        # FK model treats these as pure Z (L1=0.41621), ignoring XY
        xy_offset = math.sqrt(0.004**2 + 0.02**2)
        print(f"\nArm joint XY offset (ignored by FK): {xy_offset:.4f}m")
        print(f"  Effective L1 if including XY: {math.sqrt(0.004**2 + 0.02**2 + 0.41621**2):.5f}")
        print(f"  FK model L1: {L1}")


class TestElbowPositionsAtHome:
    """
    At home position (URDF angles = 0, 0), compare where the URDF places
    the elbow endpoints vs where the FK model says they are.
    """

    def test_right_elbow_at_home(self):
        """Right elbow endpoint at home (urdf_right=0)."""
        urdf_pos = urdf_elbow_position('right', 0.0)
        print(f"\nRight elbow at home (URDF 3D): x={urdf_pos[0]:.4f}, "
              f"y={urdf_pos[1]:.4f}, z={urdf_pos[2]:.4f}")

        # FK model: right elbow at angle = urdf_right + right_fk_offset
        fk_angle = 0.0 + RIGHT_FK_OFFSET  # pi/4
        fk_y = l0 + L1 * math.cos(fk_angle)
        fk_z = BASE_Z + L1 * math.sin(fk_angle)
        print(f"Right elbow at home (FK 2D):   y={fk_y:.4f}, z={fk_z:.4f}")

        # The URDF Y,Z should match FK y,z
        # (URDF has a pi rotation in the base joint, so Y/Z mapping may flip)
        print(f"  URDF Y vs FK y: diff={urdf_pos[1] - fk_y:.4f}")
        print(f"  URDF Z vs FK z: diff={urdf_pos[2] - fk_z:.4f}")

    def test_left_elbow_at_home(self):
        """Left elbow endpoint at home (urdf_left=0)."""
        urdf_pos = urdf_elbow_position('left', 0.0)
        print(f"\nLeft elbow at home (URDF 3D): x={urdf_pos[0]:.4f}, "
              f"y={urdf_pos[1]:.4f}, z={urdf_pos[2]:.4f}")

        fk_angle = 0.0 + LEFT_FK_OFFSET  # 3*pi/4
        fk_y = -l0 + L1 * math.cos(fk_angle)
        fk_z = BASE_Z + L1 * math.sin(fk_angle)
        print(f"Left elbow at home (FK 2D):    y={fk_y:.4f}, z={fk_z:.4f}")

        print(f"  URDF Y vs FK y: diff={urdf_pos[1] - fk_y:.4f}")
        print(f"  URDF Z vs FK z: diff={urdf_pos[2] - fk_z:.4f}")


class TestFKvsURDFAtMultipleAngles:
    """
    For several URDF joint angles, compare the elbow endpoint positions
    between the FK model and the URDF 3D chain.

    The elbows are the part controlled by the actuated joints.  If these
    don't match, everything downstream (arm links, tip) will be wrong.
    """

    ANGLES = [
        (0.0, 0.0),
        (0.3, -0.3),
        (-0.5, 0.5),
        (0.345, 0.054),   # The actual IK result from the user's test
        (-0.054, -0.345), # Mirror case
        (1.0, -1.0),
    ]

    @pytest.mark.parametrize("urdf_right,urdf_left", ANGLES)
    def test_right_elbow_fk_vs_urdf(self, urdf_right, urdf_left):
        """Compare right elbow position: FK model vs URDF chain."""
        # URDF 3D
        urdf_pos = urdf_elbow_position('right', urdf_right)

        # FK model
        fk_right = urdf_right + RIGHT_FK_OFFSET
        fk_y = l0 + L1 * math.cos(fk_right)
        fk_z = BASE_Z + L1 * math.sin(fk_right)

        print(f"\nR_urdf={urdf_right:.3f}: "
              f"URDF(y={urdf_pos[1]:.4f}, z={urdf_pos[2]:.4f}) vs "
              f"FK(y={fk_y:.4f}, z={fk_z:.4f}) "
              f"diff_y={urdf_pos[1]-fk_y:.4f} diff_z={urdf_pos[2]-fk_z:.4f}")

    @pytest.mark.parametrize("urdf_right,urdf_left", ANGLES)
    def test_left_elbow_fk_vs_urdf(self, urdf_right, urdf_left):
        """Compare left elbow position: FK model vs URDF chain."""
        urdf_pos = urdf_elbow_position('left', urdf_left)

        fk_left = urdf_left + LEFT_FK_OFFSET
        fk_y = -l0 + L1 * math.cos(fk_left)
        fk_z = BASE_Z + L1 * math.sin(fk_left)

        print(f"\nL_urdf={urdf_left:.3f}: "
              f"URDF(y={urdf_pos[1]:.4f}, z={urdf_pos[2]:.4f}) vs "
              f"FK(y={fk_y:.4f}, z={fk_z:.4f}) "
              f"diff_y={urdf_pos[1]-fk_y:.4f} diff_z={urdf_pos[2]-fk_z:.4f}")


class TestL2Mismatch:
    """
    Quantify the impact of the L2 mismatch (FK=0.65 vs URDF=0.6225).

    For the specific test case (y=-0.115, z=0.841), how far off would
    the tip be if the real arm link is 0.6225m instead of 0.65m?
    """

    def test_tip_error_at_target(self):
        """
        IK computes angles assuming L2=0.65.
        If the real arm is 0.6225m, the tip lands at a different position.
        """
        target_y, target_z = -0.115, 0.841

        # IK with L2=0.65 (current model)
        urdf_r, urdf_l = full_ik(target_y, target_z)
        # Verify
        y_verify, z_verify = full_fk(urdf_r, urdf_l)
        print(f"\nTarget: y={target_y}, z={target_z}")
        print(f"IK joints: R={urdf_r:.4f}, L={urdf_l:.4f}")
        print(f"FK verify (L2={L2}): y={y_verify:.4f}, z={z_verify:.4f}")

        # Now compute FK with L2=0.6225 (URDF actual)
        L2_urdf = 0.6225
        fk_left_angle, fk_right_angle = urdf_to_fk(urdf_l, urdf_r)

        # Redo FK with real L2
        y_l = -l0 + L1 * math.cos(fk_left_angle)
        z_l = BASE_Z + L1 * math.sin(fk_left_angle)
        y_r = l0 + L1 * math.cos(fk_right_angle)
        z_r = BASE_Z + L1 * math.sin(fk_right_angle)

        dy = y_l - y_r
        dz = z_l - z_r
        d = math.hypot(dy, dz)

        if d > 2.0 * L2_urdf:
            print(f"\n  *** ELBOWS TOO FAR APART for L2={L2_urdf}! ***")
            print(f"  Elbow distance d={d:.4f}, max reach 2*L2_urdf={2*L2_urdf:.4f}")
            print(f"  The joint angles computed with L2={L2} are UNREACHABLE "
                  f"with the real arm length {L2_urdf}")
            return

        a = d / 2.0
        h = math.sqrt(L2_urdf**2 - a**2)
        my = y_r + a * dy / d
        mz = z_r + a * dz / d
        y_real = my + h * dz / d
        z_real = mz - h * dy / d

        error_y = y_real - target_y
        error_z = z_real - target_z
        error_total = math.hypot(error_y, error_z)

        print(f"FK with L2={L2_urdf} (URDF real): y={y_real:.4f}, z={z_real:.4f}")
        print(f"  Error: dy={error_y:.4f}m, dz={error_z:.4f}m, "
              f"total={error_total:.4f}m ({error_total*100:.1f}cm)")

    def test_tip_error_across_workspace(self):
        """Check L2 mismatch error across multiple workspace points."""
        L2_urdf = 0.6225

        print(f"\n{'target_y':>10} {'target_z':>10} | {'real_y':>10} {'real_z':>10} | "
              f"{'err_y':>8} {'err_z':>8} {'err_cm':>8}")
        print("-" * 80)

        for target_y in [-0.2, -0.1, 0.0, 0.1, 0.2]:
            for target_z in [0.4, 0.6, 0.8]:
                urdf_r, urdf_l = full_ik(target_y, target_z)
                fk_left_angle, fk_right_angle = urdf_to_fk(urdf_l, urdf_r)

                y_l = -l0 + L1 * math.cos(fk_left_angle)
                z_l = BASE_Z + L1 * math.sin(fk_left_angle)
                y_r = l0 + L1 * math.cos(fk_right_angle)
                z_r = BASE_Z + L1 * math.sin(fk_right_angle)

                dy = y_l - y_r
                dz = z_l - z_r
                d = math.hypot(dy, dz)

                if d > 2.0 * L2_urdf:
                    print(f"{target_y:10.3f} {target_z:10.3f} | "
                          f"{'UNREACHABLE':>10} {'':>10} | "
                          f"{'---':>8} {'---':>8} {'---':>8}")
                    continue

                a = d / 2.0
                h = math.sqrt(L2_urdf**2 - a**2)
                my = y_r + a * dy / d
                mz = z_r + a * dz / d
                y_real = my + h * dz / d
                z_real = mz - h * dy / d

                err_y = y_real - target_y
                err_z = z_real - target_z
                err_cm = math.hypot(err_y, err_z) * 100

                print(f"{target_y:10.3f} {target_z:10.3f} | "
                      f"{y_real:10.4f} {z_real:10.4f} | "
                      f"{err_y:8.4f} {err_z:8.4f} {err_cm:7.1f}cm")
