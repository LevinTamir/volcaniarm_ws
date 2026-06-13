#ifndef VOLCANIARM_IK_PLUGIN__VOLCANIARM_IK_PLUGIN_HPP_
#define VOLCANIARM_IK_PLUGIN__VOLCANIARM_IK_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "moveit/kinematics_base/kinematics_base.hpp"
#include "moveit/robot_model/robot_model.hpp"

#include "volcaniarm_kinematics/params.hpp"

namespace volcaniarm_ik_plugin
{

// Analytic position-only IK plugin for the Volcaniarm planar five-bar. Backs
// MoveIt with volcaniarm_kinematics: the EE (closure point) Y-Z determines the
// two actuated elbow angles (inverseEE), and the three passive joint angles are
// solved deterministically (passiveAngles). The geometry is read from the
// RobotModel so the URDF stays the single source of truth.
class VolcaniarmIKPlugin : public kinematics::KinematicsBase
{
public:
  VolcaniarmIKPlugin() = default;

  bool initialize(
    const rclcpp::Node::SharedPtr & node, const moveit::core::RobotModel & robot_model,
    const std::string & group_name, const std::string & base_frame,
    const std::vector<std::string> & tip_frames, double search_discretization) override;

  bool getPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options =
      kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, std::vector<double> & solution,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options =
      kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, const std::vector<double> & consistency_limits,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options =
      kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, std::vector<double> & solution, const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options =
      kinematics::KinematicsQueryOptions()) const override;

  bool searchPositionIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    double timeout, const std::vector<double> & consistency_limits,
    std::vector<double> & solution, const IKCallbackFn & solution_callback,
    moveit_msgs::msg::MoveItErrorCodes & error_code,
    const kinematics::KinematicsQueryOptions & options =
      kinematics::KinematicsQueryOptions()) const override;

  bool getPositionFK(
    const std::vector<std::string> & link_names, const std::vector<double> & joint_angles,
    std::vector<geometry_msgs::msg::Pose> & poses) const override;

  const std::vector<std::string> & getJointNames() const override;
  const std::vector<std::string> & getLinkNames() const override;

private:
  // Core analytic solve shared by all IK entry points. Honours an optional
  // collision/validity callback (returns only a solution the callback accepts).
  bool solveIK(
    const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
    std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
    const IKCallbackFn & solution_callback) const;

  const moveit::core::JointModelGroup * jmg_{nullptr};
  std::vector<std::string> joint_names_;  // group variable order (matches seed/solution)
  std::vector<std::string> link_names_;
  volcaniarm_kinematics::Params params_;

  // Indices of the five loop joints within joint_names_.
  int idx_left_elbow_{-1};
  int idx_right_elbow_{-1};
  int idx_left_arm_{-1};
  int idx_right_arm_{-1};
  int idx_closure_{-1};
};

}  // namespace volcaniarm_ik_plugin

#endif  // VOLCANIARM_IK_PLUGIN__VOLCANIARM_IK_PLUGIN_HPP_
