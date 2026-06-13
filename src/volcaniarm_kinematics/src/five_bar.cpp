#include "volcaniarm_kinematics/five_bar.hpp"

#include <cmath>

namespace volcaniarm_kinematics
{

Point2 elbowTip(
  const Params & p, double elbow_rpy, double theta, double shoulder_y, double lateral)
{
  const double alpha = elbow_rpy + theta;
  // Arm-joint origin (lateral, L1) in the elbow frame, rotated about X by alpha.
  Point2 tip;
  tip.y = shoulder_y + lateral * std::cos(alpha) - p.L1 * std::sin(alpha);
  tip.z = p.base_z + lateral * std::sin(alpha) + p.L1 * std::cos(alpha);
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
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_left, -p.l0, p.arm_lateral);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_right, p.l0, -p.arm_lateral);

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
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_left, -p.l0, p.arm_lateral);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_right, p.l0, -p.arm_lateral);

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
  const Params & p, double elbow_rpy, double shoulder_y, double lateral,
  double y_tip, double z_tip, double theta_seed)
{
  // Invert elbowTip(): (y_tip - shoulder_y, z_tip - base_z) is the vector
  // (lateral, L1) rotated about X by alpha = elbow_rpy + theta. Rotation adds
  // angles, so alpha = atan2(dz, dy) - atan2(L1, lateral).
  const double dy = y_tip - shoulder_y;
  const double dz = z_tip - p.base_z;
  const double alpha = std::atan2(dz, dy) - std::atan2(p.L1, lateral);
  double theta = alpha - elbow_rpy;

  // Wrap onto the 2*pi branch nearest the seed for continuity along a sweep.
  const double two_pi = 2.0 * M_PI;
  theta += two_pi * std::round((theta_seed - theta) / two_pi);
  return theta;
}

double closureMargin(const Params & p, double theta_L, double theta_R)
{
  const Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta_L, -p.l0, p.arm_lateral);
  const Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta_R, p.l0, -p.arm_lateral);
  const double d = std::hypot(tip_l.y - tip_r.y, tip_l.z - tip_r.z);
  return 2.0 * p.L2 - d;
}

namespace
{
// Solve one side's elbow angle from a target EE point. The arm joint lies on
// circle(shoulder, L1eff) and circle(ee, L2); both intersections are mapped back
// to an elbow angle via sideIK and the one nearest the seed is returned.
bool sideElbowFromEE(
  const Params & p, double elbow_rpy, double shoulder_y, double lateral,
  double ee_y, double ee_z, double seed, double & theta_out)
{
  const double sy = shoulder_y;
  const double sz = p.base_z;
  const double r0 = std::hypot(p.L1, lateral);  // L1eff: |arm-joint - shoulder|
  const double r1 = p.L2;                        // |arm-joint - ee|

  const double dy = ee_y - sy;
  const double dz = ee_z - sz;
  const double d = std::hypot(dy, dz);
  if (d > r0 + r1 || d < std::abs(r0 - r1) || d < 1e-12) {
    return false;  // circles do not intersect: unreachable
  }

  const double a = (r0 * r0 - r1 * r1 + d * d) / (2.0 * d);
  const double h = std::sqrt(std::max(0.0, r0 * r0 - a * a));
  // Foot of the perpendicular from the shoulder toward the EE.
  const double my = sy + a * dy / d;
  const double mz = sz + a * dz / d;
  // The two arm-joint candidates, offset along the perpendicular.
  const double ox = h * (-dz) / d;
  const double oy = h * (dy) / d;

  const double cand_y[2] = {my + ox, my - ox};
  const double cand_z[2] = {mz + oy, mz - oy};

  double best = 0.0;
  double best_err = -1.0;
  for (int i = 0; i < 2; ++i) {
    const double th = sideIK(p, elbow_rpy, sy, lateral, cand_y[i], cand_z[i], seed);
    const double err = std::abs(th - seed);
    if (best_err < 0.0 || err < best_err) {
      best_err = err;
      best = th;
    }
  }
  theta_out = best;
  return true;
}
}  // namespace

ElbowAngles inverseEE(
  const Params & p, double ee_y, double ee_z, double seed_left, double seed_right)
{
  ElbowAngles out{seed_left, seed_right, false};
  double tl = seed_left, tr = seed_right;
  if (!sideElbowFromEE(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, ee_y, ee_z, seed_left, tl)) {
    return out;
  }
  if (!sideElbowFromEE(p, p.right_elbow_rpy, p.l0, -p.arm_lateral, ee_y, ee_z, seed_right, tr)) {
    return out;
  }
  out.theta_left = tl;
  out.theta_right = tr;
  out.valid = true;
  return out;
}

}  // namespace volcaniarm_kinematics
