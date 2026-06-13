#ifndef VOLCANIARM_KINEMATICS__FIVE_BAR_HPP_
#define VOLCANIARM_KINEMATICS__FIVE_BAR_HPP_

#include "volcaniarm_kinematics/params.hpp"

// Closure math for the Volcaniarm planar five-bar linkage. This library is the
// single source of truth for the geometry; it has zero ROS dependencies so it
// can be unit-tested with plain gtest and linked by the broadcaster, a MoveIt
// plugin, and a Pinocchio CI oracle.
//
// All functions are free, stateless (taking const Params &), and allocation-free
// so they are safe to call from a real-time control loop.
//
// Convention: the linkage lives in the world Y-Z plane. "shoulder_y" is the Y
// origin of an elbow joint (-l0 for the left side, +l0 for the right). theta is
// the actuated elbow angle. Positive closureMargin() means the loop can close.
namespace volcaniarm_kinematics
{

struct Point2
{
  double y;
  double z;
};

// Result of the two-circle FK closure. valid == false when the linkage cannot
// close (elbow tips farther apart than 2*L2, or coincident); in that case y/z
// are filled with the midpoint of the two tips (matching the legacy behaviour).
struct EEResult
{
  double y;
  double z;
  bool valid;
};

struct PassiveAngles
{
  double left_arm;
  double right_arm;
  double closure;
  bool valid;
};

struct EEPose
{
  double x;
  double y;
  double z;
  bool valid;
};

// Y-Z position of one side's arm joint (the far end of the elbow link). The
// arm-joint origin is (lateral, L1) in the rotated elbow frame; pass +arm_lateral
// for the left side and -arm_lateral for the right.
Point2 elbowTip(
  const Params & p, double elbow_rpy, double theta, double shoulder_y, double lateral);

// FK loop closure: intersect the two L2 circles centred on the elbow tips and
// pick the lower intersection (the physical EE). See EEResult for the failure
// semantics.
EEResult circleIntersect(
  const Params & p, double y_l, double z_l, double y_r, double z_r);

// Solve the three passive joint angles (left_arm, right_arm, closure) from the
// two actuated elbow angles. valid mirrors circleIntersect().
PassiveAngles passiveAngles(const Params & p, double theta_left, double theta_right);

// Full EE pose in volcaniarm_base_link. x is the planar constant (0.291) plus
// the tool offset; y/z come from the loop closure.
EEPose forwardEE(
  const Params & p, double theta_left, double theta_right, double tool_offset = 0.0);

// Per-side inverse kinematics: given a desired arm-joint Y-Z and the side's
// shoulder_y / lateral offset, return the actuated angle on the branch nearest
// theta_seed. Inverse of elbowTip(); the seed resolves the 2*pi periodic
// ambiguity and gives branch continuity along a sweep.
double sideIK(
  const Params & p, double elbow_rpy, double shoulder_y, double lateral,
  double y_tip, double z_tip, double theta_seed);

// Feasibility margin: 2*L2 - distance(left_tip, right_tip). Positive means the
// loop can close. Scalar form of the guard inside circleIntersect(), exposed for
// the broadcaster's hold-last-valid check and (later) IK reachability tests.
double closureMargin(const Params & p, double theta_L, double theta_R);

}  // namespace volcaniarm_kinematics

#endif  // VOLCANIARM_KINEMATICS__FIVE_BAR_HPP_
