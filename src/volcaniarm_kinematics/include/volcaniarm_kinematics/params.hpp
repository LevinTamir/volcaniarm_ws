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
  double L1 = 0.41621;      // upper link length (elbow link) [m]
  double L2 = 0.62243;      // lower link length (arm link) [m]
  double l0 = 0.215;        // shoulder separation from centre [m]
  double base_z = 0.0632;   // elbow joint Z offset in volcaniarm_base_link [m]

  // URDF joint-frame Rx (rpy roll) offsets [rad].
  double left_elbow_rpy = 0.7854;
  double right_elbow_rpy = -0.7854;
  double left_arm_rpy = -1.7053;
  double right_arm_rpy = 1.7053;
  double closure_rpy = 1.8398;
};

}  // namespace volcaniarm_kinematics

#endif  // VOLCANIARM_KINEMATICS__PARAMS_HPP_
