#include "volcaniarm_kinematics/five_bar.hpp"

#include <cmath>

namespace volcaniarm_kinematics
{

Point2 elbowTip(const Params & p, double elbow_rpy, double theta, double shoulder_y)
{
  const double alpha = elbow_rpy + theta;
  Point2 tip;
  tip.y = shoulder_y + p.L1 * (-std::sin(alpha));
  tip.z = p.base_z + p.L1 * std::cos(alpha);
  return tip;
}

EEResult circleIntersect(
  const Params & p, double y_l, double z_l, double y_r, double z_r)
{
  const double dy = y_l - y_r;
  const double dz = z_l - z_r;
  const double d = std::hypot(dy, dz);

  EEResult res;
  if (d > 2.0 * p.L2 || d < 1e-9) {
    // Cannot close: report the midpoint and flag invalid.
    res.y = (y_l + y_r) / 2.0;
    res.z = (z_l + z_r) / 2.0;
    res.valid = false;
    return res;
  }

  const double a = d / 2.0;
  const double h = std::sqrt(std::max(0.0, p.L2 * p.L2 - a * a));

  const double my = y_r + a * dy / d;
  const double mz = z_r + a * dz / d;

  res.y = my + h * dz / d;
  res.z = mz - h * dy / d;
  res.valid = true;
  return res;
}

PassiveAngles passiveAngles(const Params & p, double theta_left, double theta_right)
{
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_left, -p.l0);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_right, p.l0);

  PassiveAngles out{0.0, 0.0, 0.0, false};

  const EEResult ee = circleIntersect(p, tip_l.y, tip_l.z, tip_r.y, tip_r.z);
  if (!ee.valid) {
    return out;
  }

  const double alpha_left = p.left_elbow_rpy + theta_left;
  const double total_left = std::atan2(-(ee.y - tip_l.y), ee.z - tip_l.z);
  out.left_arm = total_left - alpha_left - p.left_arm_rpy;

  const double alpha_right = p.right_elbow_rpy + theta_right;
  const double total_right = std::atan2(-(ee.y - tip_r.y), ee.z - tip_r.z);
  out.right_arm = total_right - alpha_right - p.right_arm_rpy;

  const double cum_left = alpha_left + p.left_arm_rpy + out.left_arm;
  const double cum_right = alpha_right + p.right_arm_rpy + out.right_arm;
  out.closure = cum_right - cum_left - p.closure_rpy;
  out.valid = true;
  return out;
}

EEPose forwardEE(
  const Params & p, double theta_left, double theta_right, double tool_offset)
{
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_left, -p.l0);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_right, p.l0);

  const EEResult ee = circleIntersect(p, tip_l.y, tip_l.z, tip_r.y, tip_r.z);

  EEPose pose;
  // EE X = midpoint of the two elbow joint X origins + arm joint X offset.
  // Left elbow at X=0.285, right at X=0.289 in volcaniarm_base_link, arm joints
  // add 0.004, giving the planar constant 0.291.
  pose.x = 0.291 + tool_offset;
  pose.y = ee.y;
  pose.z = ee.z;
  pose.valid = ee.valid;
  return pose;
}

double sideIK(
  const Params & p, double elbow_rpy, double shoulder_y,
  double y_tip, double z_tip, double theta_seed)
{
  // Invert elbowTip(): y_tip - shoulder_y = -L1*sin(alpha),
  //                    z_tip - base_z    =  L1*cos(alpha).
  const double alpha = std::atan2(-(y_tip - shoulder_y), z_tip - p.base_z);
  double theta = alpha - elbow_rpy;

  // Wrap onto the 2*pi branch nearest the seed for continuity along a sweep.
  const double two_pi = 2.0 * M_PI;
  theta += two_pi * std::round((theta_seed - theta) / two_pi);
  return theta;
}

double closureMargin(const Params & p, double theta_L, double theta_R)
{
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_L, -p.l0);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_R, p.l0);
  const double d = std::hypot(tip_l.y - tip_r.y, tip_l.z - tip_r.z);
  return 2.0 * p.L2 - d;
}

}  // namespace volcaniarm_kinematics
