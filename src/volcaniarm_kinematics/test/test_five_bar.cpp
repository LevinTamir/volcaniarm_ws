#include <cmath>

#include <gtest/gtest.h>

#include "volcaniarm_kinematics/five_bar.hpp"

using namespace volcaniarm_kinematics;

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

double dist(const Point2 & a, const Point2 & b)
{
  return std::hypot(a.y - b.y, a.z - b.z);
}
}  // namespace

// elbowTip() and sideIK() must be exact inverses (per side), including the
// arm-joint lateral offset (left +arm_lateral, right -arm_lateral).
TEST(FiveBar, ElbowTipSideIkRoundTrip)
{
  Params p;
  for (double theta = -1.4; theta <= 1.4; theta += 0.05) {
    // Left side.
    Point2 tip_l = elbowTip(p, p.left_elbow_rpy, theta, -p.l0, p.arm_lateral);
    double rec_l = sideIK(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, tip_l.y, tip_l.z, theta);
    EXPECT_NEAR(rec_l, theta, 1e-9) << "left theta=" << theta;

    // Right side.
    Point2 tip_r = elbowTip(p, p.right_elbow_rpy, theta, p.l0, -p.arm_lateral);
    double rec_r = sideIK(p, p.right_elbow_rpy, p.l0, -p.arm_lateral, tip_r.y, tip_r.z, theta);
    EXPECT_NEAR(rec_r, theta, 1e-9) << "right theta=" << theta;
  }
}

// A seed offset by a full turn must recover the angle on that turn's branch.
TEST(FiveBar, SideIkBranchSelectionFollowsSeed)
{
  Params p;
  const double theta = 0.3;
  Point2 tip = elbowTip(p, p.left_elbow_rpy, theta, -p.l0, p.arm_lateral);

  double near_zero = sideIK(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, tip.y, tip.z, theta);
  double near_plus =
    sideIK(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, tip.y, tip.z, theta + kTwoPi);
  double near_minus =
    sideIK(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, tip.y, tip.z, theta - kTwoPi);

  EXPECT_NEAR(near_zero, theta, 1e-9);
  EXPECT_NEAR(near_plus, theta + kTwoPi, 1e-9);
  EXPECT_NEAR(near_minus, theta - kTwoPi, 1e-9);
}

// In the feasible region the EE must lie exactly L2 from BOTH arm joints, and the
// passive solve must agree with forwardEE.
TEST(FiveBar, ClosureInvariantOverWorkspaceGrid)
{
  Params p;
  int feasible = 0;
  for (double tl = -1.2; tl <= 1.2; tl += 0.1) {
    for (double tr = -1.2; tr <= 1.2; tr += 0.1) {
      if (closureMargin(p, tl, tr) <= 0.0) {
        continue;
      }
      ++feasible;

      Point2 tip_l = elbowTip(p, p.left_elbow_rpy, tl, -p.l0, p.arm_lateral);
      Point2 tip_r = elbowTip(p, p.right_elbow_rpy, tr, p.l0, -p.arm_lateral);
      EEResult ee = circleIntersect(p, tip_l.y, tip_l.z, tip_r.y, tip_r.z);
      ASSERT_TRUE(ee.valid) << "tl=" << tl << " tr=" << tr;

      Point2 ee_pt{ee.y, ee.z};
      EXPECT_NEAR(dist(ee_pt, tip_l), p.L2, 1e-9);
      EXPECT_NEAR(dist(ee_pt, tip_r), p.L2, 1e-9);

      PassiveAngles pa = passiveAngles(p, tl, tr);
      EXPECT_TRUE(pa.valid);
      EEPose pose = forwardEE(p, tl, tr);
      EXPECT_TRUE(pose.valid);
      EXPECT_NEAR(pose.y, ee.y, 1e-12);
      EXPECT_NEAR(pose.z, ee.z, 1e-12);
    }
  }
  EXPECT_GT(feasible, 0) << "grid produced no feasible poses; check ranges";
}

// closureMargin is the signed slack 2*L2 - d, and circleIntersect.valid must
// flip exactly at the zero crossing.
TEST(FiveBar, ClosureMarginMatchesFeasibilityBoundary)
{
  Params p;
  for (double tl = -1.4; tl <= 1.4; tl += 0.07) {
    for (double tr = -1.4; tr <= 1.4; tr += 0.07) {
      double margin = closureMargin(p, tl, tr);
      Point2 tip_l = elbowTip(p, p.left_elbow_rpy, tl, -p.l0, p.arm_lateral);
      Point2 tip_r = elbowTip(p, p.right_elbow_rpy, tr, p.l0, -p.arm_lateral);
      double d = dist(tip_l, tip_r);
      EXPECT_NEAR(margin, 2.0 * p.L2 - d, 1e-12);

      EEResult ee = circleIntersect(p, tip_l.y, tip_l.z, tip_r.y, tip_r.z);
      // Away from the razor edge, valid must agree with a positive margin.
      if (margin > 1e-6) {
        EXPECT_TRUE(ee.valid) << "tl=" << tl << " tr=" << tr << " margin=" << margin;
      } else if (margin < -1e-6) {
        EXPECT_FALSE(ee.valid) << "tl=" << tl << " tr=" << tr << " margin=" << margin;
      }
    }
  }
}

// Pushing the arm joints apart drives the margin negative and the solve invalid;
// the degenerate EEResult must fall back to the tip midpoint.
TEST(FiveBar, InfeasibleFallsBackToMidpoint)
{
  Params p;
  // The arm joints spread farthest apart near alpha = +/- pi/2, i.e. tl = +0.7854
  // and tr = -0.7854 (left tip swings to -Y, right tip to +Y). This is the only
  // region where the loop cannot close (a thin sliver: max deficit ~17 mm).
  const double tl = M_PI / 2.0 - p.left_elbow_rpy;   // ~0.7854
  const double tr = -M_PI / 2.0 - p.right_elbow_rpy;  // ~-0.7854
  ASSERT_LT(closureMargin(p, tl, tr), 0.0);

  Point2 tip_l = elbowTip(p, p.left_elbow_rpy, tl, -p.l0, p.arm_lateral);
  Point2 tip_r = elbowTip(p, p.right_elbow_rpy, tr, p.l0, -p.arm_lateral);
  EEResult ee = circleIntersect(p, tip_l.y, tip_l.z, tip_r.y, tip_r.z);
  EXPECT_FALSE(ee.valid);
  EXPECT_NEAR(ee.y, (tip_l.y + tip_r.y) / 2.0, 1e-12);
  EXPECT_NEAR(ee.z, (tip_l.z + tip_r.z) / 2.0, 1e-12);

  PassiveAngles pa = passiveAngles(p, tl, tr);
  EXPECT_FALSE(pa.valid);
}

// Branch continuity: sweeping theta while seeding with the previous solution must
// produce a smooth, jump-free sequence that matches the analytic angle.
TEST(FiveBar, BranchContinuityAlongSweep)
{
  Params p;
  double prev = -1.3;
  for (double theta = -1.3; theta <= 1.3; theta += 0.02) {
    Point2 tip = elbowTip(p, p.left_elbow_rpy, theta, -p.l0, p.arm_lateral);
    double rec = sideIK(p, p.left_elbow_rpy, -p.l0, p.arm_lateral, tip.y, tip.z, prev);
    EXPECT_NEAR(rec, theta, 1e-9);
    EXPECT_LT(std::abs(rec - prev), 0.1) << "discontinuity at theta=" << theta;
    prev = rec;
  }
}

// forwardEE -> inverseEE round-trip: seeding with the true angles must recover
// the actuated elbow angles exactly across the feasible workspace.
TEST(FiveBar, ForwardInverseEeRoundTrip)
{
  Params p;
  int feasible = 0;
  for (double tl = -1.0; tl <= 1.0; tl += 0.05) {
    for (double tr = -1.0; tr <= 1.0; tr += 0.05) {
      if (closureMargin(p, tl, tr) <= 1e-3) {
        continue;
      }
      EEPose pose = forwardEE(p, tl, tr);
      if (!pose.valid) {
        continue;
      }
      ++feasible;
      ElbowAngles ik = inverseEE(p, pose.y, pose.z, tl, tr);
      ASSERT_TRUE(ik.valid) << "tl=" << tl << " tr=" << tr;
      EXPECT_NEAR(ik.theta_left, tl, 1e-9) << "tl=" << tl << " tr=" << tr;
      EXPECT_NEAR(ik.theta_right, tr, 1e-9) << "tl=" << tl << " tr=" << tr;
    }
  }
  EXPECT_GT(feasible, 0);
}

// A target well outside the reachable workspace returns valid == false.
TEST(FiveBar, InverseEeUnreachable)
{
  Params p;
  // Far below/away from the linkage: no circle intersection on either side.
  ElbowAngles ik = inverseEE(p, 0.0, 5.0, 0.0, 0.0);
  EXPECT_FALSE(ik.valid);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
