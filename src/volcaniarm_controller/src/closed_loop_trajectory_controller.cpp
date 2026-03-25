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
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_init();
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Passive joint configuration
  auto_declare<std::vector<std::string>>("passive_joints", std::vector<std::string>{});
  auto_declare<std::string>("left_elbow_joint", "volcaniarm_left_elbow_joint");
  auto_declare<std::string>("right_elbow_joint", "volcaniarm_right_elbow_joint");

  // Kinematic parameters
  auto_declare<double>("kinematics.L1", 0.41621);
  auto_declare<double>("kinematics.L2", 0.65);
  auto_declare<double>("kinematics.l0", 0.215);
  auto_declare<double>("kinematics.base_z", 0.0632);

  // URDF joint frame Rx offsets
  auto_declare<double>("kinematics.left_elbow_rpy", 0.7854);
  auto_declare<double>("kinematics.right_elbow_rpy", -0.7854);
  auto_declare<double>("kinematics.left_arm_rpy", -1.7053);
  auto_declare<double>("kinematics.right_arm_rpy", 1.7053);
  auto_declare<double>("kinematics.closure_rpy", 1.8398);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ClosedLoopTrajectoryController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = joint_trajectory_controller::JointTrajectoryController::on_configure(previous_state);
  if (ret != controller_interface::CallbackReturn::SUCCESS) {
    return ret;
  }

  // Read parameters
  passive_joint_names_ = get_node()->get_parameter("passive_joints").as_string_array();
  left_elbow_joint_name_ = get_node()->get_parameter("left_elbow_joint").as_string();
  right_elbow_joint_name_ = get_node()->get_parameter("right_elbow_joint").as_string();

  L1_ = get_node()->get_parameter("kinematics.L1").as_double();
  L2_ = get_node()->get_parameter("kinematics.L2").as_double();
  l0_ = get_node()->get_parameter("kinematics.l0").as_double();
  base_z_ = get_node()->get_parameter("kinematics.base_z").as_double();

  left_elbow_rpy_ = get_node()->get_parameter("kinematics.left_elbow_rpy").as_double();
  right_elbow_rpy_ = get_node()->get_parameter("kinematics.right_elbow_rpy").as_double();
  left_arm_rpy_ = get_node()->get_parameter("kinematics.left_arm_rpy").as_double();
  right_arm_rpy_ = get_node()->get_parameter("kinematics.right_arm_rpy").as_double();
  closure_rpy_ = get_node()->get_parameter("kinematics.closure_rpy").as_double();

  if (passive_joint_names_.empty()) {
    RCLCPP_WARN(get_node()->get_logger(),
      "No passive_joints configured — passive joint states will not be published.");
  }

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
  auto ret = joint_trajectory_controller::JointTrajectoryController::update(time, period);

  if (!passive_joint_names_.empty() && passive_joint_pub_) {
    double theta_left = 0.0;
    double theta_right = 0.0;
    bool found_left = false;
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

void ClosedLoopTrajectoryController::elbow_tip(
  double elbow_rpy, double theta, double shoulder_y,
  double & y_out, double & z_out) const
{
  // Total rotation around X for the elbow link
  double alpha = elbow_rpy + theta;
  // Local Z axis in volcaniarm_base_link: [0, -sin(alpha), cos(alpha)]
  // Elbow tip = shoulder_origin + L1 * local_Z
  y_out = shoulder_y + L1_ * (-std::sin(alpha));
  z_out = base_z_ + L1_ * std::cos(alpha);
}

bool ClosedLoopTrajectoryController::compute_ee(
  double y_l, double z_l, double y_r, double z_r,
  double & y_ee, double & z_ee) const
{
  double dy = y_l - y_r;
  double dz = z_l - z_r;
  double d = std::hypot(dy, dz);

  if (d > 2.0 * L2_ || d < 1e-9) {
    y_ee = (y_l + y_r) / 2.0;
    z_ee = (z_l + z_r) / 2.0;
    return false;
  }

  double a = d / 2.0;
  double h = std::sqrt(std::max(0.0, L2_ * L2_ - a * a));

  double my = y_r + a * dy / d;
  double mz = z_r + a * dz / d;

  // Choose the solution where arm extends in +Z direction (downward in world)
  y_ee = my + h * dz / d;
  z_ee = mz - h * dy / d;

  return true;
}

void ClosedLoopTrajectoryController::compute_passive_angles(
  double theta_left, double theta_right,
  double & left_arm_out, double & right_arm_out, double & closure_out) const
{
  // Elbow tip positions in volcaniarm_base_link frame
  double y_l, z_l, y_r, z_r;
  elbow_tip(left_elbow_rpy_, theta_left, -l0_, y_l, z_l);
  elbow_tip(right_elbow_rpy_, theta_right, l0_, y_r, z_r);

  // End-effector position (circle-circle intersection)
  double y_ee, z_ee;
  compute_ee(y_l, z_l, y_r, z_r, y_ee, z_ee);

  // For each arm link, the total Rx rotation from volcaniarm_base_link to
  // the arm link frame is: elbow_rpy + theta_elbow + arm_rpy + theta_arm
  //
  // The arm link's local Z axis in volcaniarm_base_link is:
  //   [0, -sin(total_rot), cos(total_rot)]
  //
  // This must point from elbow tip toward end-effector, so:
  //   total_rot = atan2(-(y_ee - y_elbow), z_ee - z_elbow)
  //   theta_arm = total_rot - elbow_rpy - theta_elbow - arm_rpy

  double alpha_left = left_elbow_rpy_ + theta_left;
  double total_left = std::atan2(-(y_ee - y_l), z_ee - z_l);
  left_arm_out = total_left - alpha_left - left_arm_rpy_;

  double alpha_right = right_elbow_rpy_ + theta_right;
  double total_right = std::atan2(-(y_ee - y_r), z_ee - z_r);
  right_arm_out = total_right - alpha_right - right_arm_rpy_;

  // Closure joint: the residual rotation to align left arm tip with right arm tip
  double cum_left = alpha_left + left_arm_rpy_ + left_arm_out;
  double cum_right = alpha_right + right_arm_rpy_ + right_arm_out;
  closure_out = cum_right - cum_left - closure_rpy_;
}

}  // namespace volcaniarm_controller

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_controller::ClosedLoopTrajectoryController,
  controller_interface::ControllerInterface)
