// Pinocchio CI oracle for volcaniarm_kinematics.
//
// Independent check that the analytic closed-form solver reproduces the loop
// closure of the *actual URDF*: for a grid of actuated elbow angles, the
// analytic passive angles are fed through Pinocchio forward kinematics on the
// model built from the (live, xacro-expanded) URDF, and the two cut frames
// (closure_dummy_link on the left branch, right_arm_tip_link on the right) must
// coincide. Because all five joint axes are X, the X coordinate is a fixed 4 mm
// offset along the pin axis (physical, not an error); the loop must close in the
// Y-Z plane. This guards the analytic constants/math against URDF drift.

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "volcaniarm_kinematics/five_bar.hpp"

#ifndef ORACLE_URDF_PATH
#error "ORACLE_URDF_PATH must be defined by the build (path to the expanded URDF)"
#endif

using namespace volcaniarm_kinematics;

namespace
{
constexpr double kYzTol = 1e-7;  // 0.1 micron; measured numerical floor ~1e-8 m

double setJoint(
  const pinocchio::Model & model, Eigen::VectorXd & q,
  const std::string & name, double value)
{
  const auto jid = model.getJointId(name);
  const int idx = model.joints[jid].idx_q();
  q[idx] = value;
  return idx;
}
}  // namespace

TEST(PinocchioOracle, AnalyticClosesUrdfLoop)
{
  pinocchio::Model model;
  ASSERT_NO_THROW(pinocchio::urdf::buildModel(std::string(ORACLE_URDF_PATH), model));
  pinocchio::Data data(model);

  ASSERT_TRUE(model.existFrame("closure_dummy_link"));
  ASSERT_TRUE(model.existFrame("right_arm_tip_link"));
  const auto f_closure = model.getFrameId("closure_dummy_link");
  const auto f_tip = model.getFrameId("right_arm_tip_link");

  Params p;  // defaults must mirror the URDF geometry
  Eigen::VectorXd q = Eigen::VectorXd::Zero(model.nq);

  double max_yz = 0.0;
  double max_x = 0.0;
  int feasible = 0;

  for (double tl = -1.0; tl <= 1.0 + 1e-9; tl += 0.05) {
    for (double tr = -1.0; tr <= 1.0 + 1e-9; tr += 0.05) {
      if (closureMargin(p, tl, tr) <= 0.0) {
        continue;
      }
      const PassiveAngles pa = passiveAngles(p, tl, tr);
      ASSERT_TRUE(pa.valid) << "tl=" << tl << " tr=" << tr;
      ++feasible;

      setJoint(model, q, "volcaniarm_left_elbow_joint", tl);
      setJoint(model, q, "volcaniarm_right_elbow_joint", tr);
      setJoint(model, q, "volcaniarm_left_arm_joint", pa.left_arm);
      setJoint(model, q, "volcaniarm_right_arm_joint", pa.right_arm);
      setJoint(model, q, "closure_joint", pa.closure);

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      const Eigen::Vector3d d =
        data.oMf[f_closure].translation() - data.oMf[f_tip].translation();
      max_yz = std::max(max_yz, std::hypot(d.y(), d.z()));
      max_x = std::max(max_x, std::abs(d.x()));

      EXPECT_LT(std::hypot(d.y(), d.z()), kYzTol)
        << "loop fails to close at tl=" << tl << " tr=" << tr;
    }
  }

  ASSERT_GT(feasible, 0) << "no feasible poses in grid";
  RecordProperty("feasible_poses", feasible);
  // X is a fixed offset along the pin axis (the two arms sit 4 mm apart on the
  // pin); assert it stays constant rather than zero.
  EXPECT_NEAR(max_x, 0.004, 5e-4) << "unexpected X spread along the pin axis";

  std::cout << "[oracle] feasible poses: " << feasible
            << "  max Y-Z residual: " << max_yz * 1e3 << " mm"
            << "  X offset: " << max_x * 1e3 << " mm" << std::endl;
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
