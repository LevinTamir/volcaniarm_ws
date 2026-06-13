#include "volcaniarm_ik_plugin/volcaniarm_ik_plugin.hpp"

#include <algorithm>
#include <cmath>

#include "moveit/robot_model/joint_model_group.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

#include "volcaniarm_kinematics/five_bar.hpp"

namespace volcaniarm_ik_plugin
{
namespace
{
constexpr char kLeftElbow[] = "volcaniarm_left_elbow_joint";
constexpr char kRightElbow[] = "volcaniarm_right_elbow_joint";
constexpr char kLeftArm[] = "volcaniarm_left_arm_joint";
constexpr char kRightArm[] = "volcaniarm_right_arm_joint";
constexpr char kClosure[] = "closure_joint";
constexpr char kExpectedBase[] = "volcaniarm_base_link";

const auto LOGGER = rclcpp::get_logger("volcaniarm_ik_plugin");

// Roll (Rx) of a pure-X origin transform.
double rollOf(const Eigen::Isometry3d & t)
{
  const Eigen::Matrix3d r = t.rotation();
  return std::atan2(r(2, 1), r(1, 1));
}
}  // namespace

bool VolcaniarmIKPlugin::initialize(
  const rclcpp::Node::SharedPtr & /*node*/, const moveit::core::RobotModel & robot_model,
  const std::string & group_name, const std::string & base_frame,
  const std::vector<std::string> & tip_frames, double search_discretization)
{
  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  jmg_ = robot_model.getJointModelGroup(group_name);
  if (jmg_ == nullptr) {
    RCLCPP_ERROR(LOGGER, "No joint model group '%s'", group_name.c_str());
    return false;
  }

  joint_names_ = jmg_->getVariableNames();
  link_names_ = jmg_->getLinkModelNames();

  auto find_idx = [this](const char * name) -> int {
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i] == name) {
        return static_cast<int>(i);
      }
    }
    return -1;
  };
  idx_left_elbow_ = find_idx(kLeftElbow);
  idx_right_elbow_ = find_idx(kRightElbow);
  idx_left_arm_ = find_idx(kLeftArm);
  idx_right_arm_ = find_idx(kRightArm);
  idx_closure_ = find_idx(kClosure);
  if (idx_left_elbow_ < 0 || idx_right_elbow_ < 0 || idx_left_arm_ < 0 ||
    idx_right_arm_ < 0 || idx_closure_ < 0)
  {
    RCLCPP_ERROR(
      LOGGER, "Group '%s' must contain the five loop joints (2 elbows, 2 arms, closure)",
      group_name.c_str());
    return false;
  }

  if (base_frame != kExpectedBase) {
    RCLCPP_WARN(
      LOGGER, "base_frame '%s' != expected '%s'; the planar Y-Z math assumes the latter",
      base_frame.c_str(), kExpectedBase);
  }

  // Derive the linkage geometry from the RobotModel (URDF single source of truth).
  // Each joint's origin is the child link's joint-origin transform.
  auto origin = [&robot_model](const char * jn) -> const Eigen::Isometry3d & {
    return robot_model.getJointModel(jn)->getChildLinkModel()->getJointOriginTransform();
  };
  const Eigen::Isometry3d & le = origin(kLeftElbow);
  const Eigen::Isometry3d & re = origin(kRightElbow);
  const Eigen::Isometry3d & la = origin(kLeftArm);
  const Eigen::Isometry3d & ra = origin(kRightArm);
  const Eigen::Isometry3d & cl = origin(kClosure);

  params_.base_z = le.translation().z();
  params_.l0 = -le.translation().y();
  params_.L1 = la.translation().z();
  params_.arm_lateral = la.translation().y();
  params_.L2 = cl.translation().z();
  params_.left_elbow_rpy = rollOf(le);
  params_.right_elbow_rpy = rollOf(re);
  params_.left_arm_rpy = rollOf(la);
  params_.right_arm_rpy = rollOf(ra);
  params_.closure_rpy = rollOf(cl);

  RCLCPP_INFO(
    LOGGER,
    "VolcaniarmIKPlugin ready: group=%s base=%s tip=%s; L1=%.5f L2=%.5f l0=%.5f "
    "base_z=%.5f arm_lateral=%.5f",
    group_name.c_str(), base_frame.c_str(),
    tip_frames.empty() ? "?" : tip_frames.front().c_str(),
    params_.L1, params_.L2, params_.l0, params_.base_z, params_.arm_lateral);
  return true;
}

bool VolcaniarmIKPlugin::solveIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const IKCallbackFn & solution_callback) const
{
  if (ik_seed_state.size() != joint_names_.size()) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    return false;
  }

  const double seed_left = ik_seed_state[idx_left_elbow_];
  const double seed_right = ik_seed_state[idx_right_elbow_];

  const auto ik = volcaniarm_kinematics::inverseEE(
    params_, ik_pose.position.y, ik_pose.position.z, seed_left, seed_right);
  if (!ik.valid) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }
  const auto pa = volcaniarm_kinematics::passiveAngles(params_, ik.theta_left, ik.theta_right);
  if (!pa.valid) {
    error_code.val = moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  solution = ik_seed_state;
  solution[idx_left_elbow_] = ik.theta_left;
  solution[idx_right_elbow_] = ik.theta_right;
  solution[idx_left_arm_] = pa.left_arm;
  solution[idx_right_arm_] = pa.right_arm;
  solution[idx_closure_] = pa.closure;

  if (solution_callback) {
    solution_callback(ik_pose, solution, error_code);
    return error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  }
  error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return true;
}

bool VolcaniarmIKPlugin::getPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & /*options*/) const
{
  return solveIK(ik_pose, ik_seed_state, solution, error_code, IKCallbackFn());
}

bool VolcaniarmIKPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double /*timeout*/, std::vector<double> & solution,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & /*options*/) const
{
  return solveIK(ik_pose, ik_seed_state, solution, error_code, IKCallbackFn());
}

bool VolcaniarmIKPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double /*timeout*/, const std::vector<double> & /*consistency_limits*/,
  std::vector<double> & solution, moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & /*options*/) const
{
  return solveIK(ik_pose, ik_seed_state, solution, error_code, IKCallbackFn());
}

bool VolcaniarmIKPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double /*timeout*/, std::vector<double> & solution, const IKCallbackFn & solution_callback,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & /*options*/) const
{
  return solveIK(ik_pose, ik_seed_state, solution, error_code, solution_callback);
}

bool VolcaniarmIKPlugin::searchPositionIK(
  const geometry_msgs::msg::Pose & ik_pose, const std::vector<double> & ik_seed_state,
  double /*timeout*/, const std::vector<double> & /*consistency_limits*/,
  std::vector<double> & solution, const IKCallbackFn & solution_callback,
  moveit_msgs::msg::MoveItErrorCodes & error_code,
  const kinematics::KinematicsQueryOptions & /*options*/) const
{
  return solveIK(ik_pose, ik_seed_state, solution, error_code, solution_callback);
}

bool VolcaniarmIKPlugin::getPositionFK(
  const std::vector<std::string> & link_names, const std::vector<double> & joint_angles,
  std::vector<geometry_msgs::msg::Pose> & poses) const
{
  if (joint_angles.size() != joint_names_.size()) {
    return false;
  }
  // Delegate to MoveIt's own FK so the tip pose is exactly consistent with the
  // RobotModel; express results in the base frame.
  moveit::core::RobotState state(robot_model_);
  state.setToDefaultValues();
  state.setJointGroupPositions(jmg_, joint_angles);
  state.update();

  const Eigen::Isometry3d base_inv = state.getGlobalLinkTransform(base_frame_).inverse();
  poses.clear();
  poses.reserve(link_names.size());
  for (const auto & ln : link_names) {
    poses.push_back(tf2::toMsg(base_inv * state.getGlobalLinkTransform(ln)));
  }
  return true;
}

const std::vector<std::string> & VolcaniarmIKPlugin::getJointNames() const
{
  return joint_names_;
}

const std::vector<std::string> & VolcaniarmIKPlugin::getLinkNames() const
{
  return link_names_;
}

}  // namespace volcaniarm_ik_plugin

PLUGINLIB_EXPORT_CLASS(volcaniarm_ik_plugin::VolcaniarmIKPlugin, kinematics::KinematicsBase)
