#include "volcaniarm_controller/closed_loop_trajectory_controller.hpp"

#include <cmath>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace volcaniarm_controller
{

controller_interface::CallbackReturn
ClosedLoopTrajectoryController::on_init()
{
  // Initialise the parent JointTrajectoryController first
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Declare additional parameters for the closed-loop passive joints
  auto_declare<std::vector<std::string>>(
    "passive_joints", std::vector<std::string>{});
  auto_declare<std::string>("left_elbow_joint", "volcaniarm_left_elbow_joint");
  auto_declare<std::string>("right_elbow_joint", "volcaniarm_right_elbow_joint");

  // Kinematic parameters
  auto_declare<double>("kinematics.L1", 0.41621);
  auto_declare<double>("kinematics.L2", 0.65);
  auto_declare<double>("kinematics.l0", 0.215);
  auto_declare<double>("kinematics.base_z", 0.0582);
  auto_declare<double>("kinematics.cum_left_base", 2.2217);
  auto_declare<double>("kinematics.cum_right_base", 4.0615);
  auto_declare<double>("kinematics.closure_rpy", 1.8398);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ClosedLoopTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  // Configure the parent first
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Read our additional parameters
  passive_joint_names_ = get_node()->get_parameter("passive_joints")
                           .as_string_array();
  left_elbow_joint_name_ = get_node()->get_parameter("left_elbow_joint")
                             .as_string();
  right_elbow_joint_name_ = get_node()->get_parameter("right_elbow_joint")
                              .as_string();

  L1_  = get_node()->get_parameter("kinematics.L1").as_double();
  L2_  = get_node()->get_parameter("kinematics.L2").as_double();
  l0_  = get_node()->get_parameter("kinematics.l0").as_double();
  base_z_ = get_node()->get_parameter("kinematics.base_z").as_double();
  cum_left_base_  = get_node()->get_parameter("kinematics.cum_left_base").as_double();
  cum_right_base_ = get_node()->get_parameter("kinematics.cum_right_base").as_double();
  closure_rpy_ = get_node()->get_parameter("kinematics.closure_rpy").as_double();

  if (passive_joint_names_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
      "No passive_joints configured — passive joint states will not be published.");
  }

  // Compute home-position calibration offsets (elbow angles = 0)
  auto [y_ee, z_ee] = fk(0.0, 0.0);
  double y_l = -l0_ + L1_;  // cos(0)=1
  double z_l = base_z_;      // sin(0)=0
  double y_r =  l0_ + L1_;
  double z_r = base_z_;
  left_arm_home_offset_  = std::atan2(y_ee - y_l, z_ee - z_l);
  right_arm_home_offset_ = std::atan2(y_ee - y_r, z_ee - z_r);

  // Create publisher for passive joint states
  passive_joint_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "/joint_states", 10);

  RCLCPP_INFO(get_node()->get_logger(),
    "ClosedLoopTrajectoryController configured with %zu passive joints",
    passive_joint_names_.size());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ClosedLoopTrajectoryController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  return joint_trajectory_controller::JointTrajectoryController::on_activate(previous_state);
}

controller_interface::return_type
ClosedLoopTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // Run the parent trajectory controller update (commands, interpolation, etc.)
  auto ret = joint_trajectory_controller::JointTrajectoryController::update(time, period);

  // Publish passive joint states regardless of the parent's return value
  if (!passive_joint_names_.empty() && passive_joint_pub_) {
    // Read current elbow positions from the state interfaces
    double theta_left  = 0.0;
    double theta_right = 0.0;
    bool found_left  = false;
    bool found_right = false;

    for (size_t i = 0; i < dof_; ++i) {
      const auto & joint_name = params_.joints[i];
      if (has_position_state_interface_) {
        auto pos_opt = joint_state_interface_[0][i].get().get_optional();
        if (!pos_opt.has_value()) { continue; }
        double pos = pos_opt.value();
        if (joint_name == left_elbow_joint_name_) {
          theta_left = pos;
          found_left = true;
        } else if (joint_name == right_elbow_joint_name_) {
          theta_right = pos;
          found_right = true;
        }
      }
    }

    if (found_left && found_right) {
      double left_arm = 0.0, right_arm = 0.0, closure = 0.0;
      compute_passive_angles(theta_left, theta_right, left_arm, right_arm, closure);

      sensor_msgs::msg::JointState js;
      js.header.stamp = time;
      js.name = passive_joint_names_;
      js.position = {left_arm, right_arm, closure};
      passive_joint_pub_->publish(js);
    }
  }

  return ret;
}

std::pair<double, double>
ClosedLoopTrajectoryController::fk(double theta_right, double theta_left) const
{
  // Elbow tip positions in the planar YZ model
  double y_l = -l0_ + L1_ * std::cos(theta_left);
  double z_l = base_z_ + L1_ * std::sin(theta_left);

  double y_r =  l0_ + L1_ * std::cos(theta_right);
  double z_r = base_z_ + L1_ * std::sin(theta_right);

  // Circle-circle intersection (both circles have radius L2)
  double dy = y_l - y_r;
  double dz = z_l - z_r;
  double d  = std::hypot(dy, dz);

  if (d > 2.0 * L2_ || d < 1e-9) {
    // Degenerate — return midpoint
    return {(y_l + y_r) / 2.0, (z_l + z_r) / 2.0};
  }

  double a = d / 2.0;
  double h = std::sqrt(std::max(0.0, L2_ * L2_ - a * a));

  double my = y_r + a * dy / d;
  double mz = z_r + a * dz / d;

  // Choose the "lower" solution (end-effector below elbows)
  double y_ee = my + h * dz / d;
  double z_ee = mz - h * dy / d;

  return {y_ee, z_ee};
}

void ClosedLoopTrajectoryController::compute_passive_angles(
  double theta_left, double theta_right,
  double & left_arm_out, double & right_arm_out, double & closure_out) const
{
  auto [y_ee, z_ee] = fk(theta_right, theta_left);

  // Elbow tip positions
  double y_l = -l0_ + L1_ * std::cos(theta_left);
  double z_l = base_z_ + L1_ * std::sin(theta_left);
  double y_r =  l0_ + L1_ * std::cos(theta_right);
  double z_r = base_z_ + L1_ * std::sin(theta_right);

  // Arm direction angles in the planar world frame
  double left_arm_world  = std::atan2(y_ee - y_l, z_ee - z_l);
  double right_arm_world = std::atan2(y_ee - y_r, z_ee - z_r);

  // Passive joint angle = world angle change from home position
  left_arm_out  = left_arm_world  - left_arm_home_offset_;
  right_arm_out = right_arm_world - right_arm_home_offset_;

  // Closure joint: aligns left chain tip frame with right chain tip frame
  double cum_left  = cum_left_base_  + theta_left  + left_arm_out;
  double cum_right = cum_right_base_ + theta_right + right_arm_out;
  closure_out = cum_right - cum_left - closure_rpy_;
}

}  // namespace volcaniarm_controller

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_controller::ClosedLoopTrajectoryController,
  controller_interface::ControllerInterface)
