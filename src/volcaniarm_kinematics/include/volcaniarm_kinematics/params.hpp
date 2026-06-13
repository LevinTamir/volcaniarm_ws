#ifndef VOLCANIARM_KINEMATICS__PARAMS_HPP_
#define VOLCANIARM_KINEMATICS__PARAMS_HPP_

namespace volcaniarm_kinematics
{

// Geometric parameters of the planar five-bar linkage. Defaults match the
// authoritative values in volcaniarm_controllers.yaml (NOT the stale C++
// defaults that used to live in the controller, e.g. L2 was 0.65 there).
//
// Phase 1: loaded from ROS parameters by the broadcaster. A later phase
// derives these from robot_description so the URDF is the single source.
struct Params
{
  double L1 = 0.41621;      // elbow-link length: arm-joint Z offset in elbow frame [m]
  double L2 = 0.6225;       // arm-link length: closure/tip Z offset in arm frame [m]
  double l0 = 0.215;        // shoulder separation from centre [m]
  double base_z = 0.0632;   // elbow joint Z offset in volcaniarm_base_link [m]

  // Arm-joint lateral (Y) offset in the elbow-link frame [m]. The URDF mounts
  // the two arms to opposite sides of their elbow links (left +, right -), so
  // the arm-joint origin is (arm_lateral, L1) in the rotated elbow frame, not a
  // pure L1 along it. Including this makes the analytic model reproduce the URDF
  // loop closure exactly (validated by the Pinocchio oracle to ~1e-8 m).
  double arm_lateral = 0.02;

  // URDF joint-frame Rx (rpy roll) offsets [rad].
  double left_elbow_rpy = 0.7854;
  double right_elbow_rpy = -0.7854;
  double left_arm_rpy = -1.7053;
  double right_arm_rpy = 1.7053;
  double closure_rpy = 1.8398;
};

}  // namespace volcaniarm_kinematics

#endif  // VOLCANIARM_KINEMATICS__PARAMS_HPP_
