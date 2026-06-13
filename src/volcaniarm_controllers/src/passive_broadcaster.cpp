#include "volcaniarm_controllers/passive_broadcaster.hpp"

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace volcaniarm_controller
{

using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;

controller_interface::CallbackReturn PassiveBroadcaster::on_init()
{
  // Joint configuration.
  auto_declare<std::string>("left_elbow_joint", "volcaniarm_left_elbow_joint");
  auto_declare<std::string>("right_elbow_joint", "volcaniarm_right_elbow_joint");
  auto_declare<std::vector<std::string>>("passive_joints", std::vector<std::string>{});

  // EE marker visualization.
  auto_declare<bool>("ee_marker.enable", true);
  auto_declare<std::string>("ee_marker.frame_id", "volcaniarm_base_link");
  auto_declare<double>("ee_marker.tool_offset", 0.0);
  auto_declare<int>("ee_marker.trail_size", 300);

  // Kinematic parameters (defaults match volcaniarm_kinematics::Params).
  volcaniarm_kinematics::Params d;
  auto_declare<double>("kinematics.L1", d.L1);
  auto_declare<double>("kinematics.L2", d.L2);
  auto_declare<double>("kinematics.l0", d.l0);
  auto_declare<double>("kinematics.base_z", d.base_z);
  auto_declare<double>("kinematics.left_elbow_rpy", d.left_elbow_rpy);
  auto_declare<double>("kinematics.right_elbow_rpy", d.right_elbow_rpy);
  auto_declare<double>("kinematics.left_arm_rpy", d.left_arm_rpy);
  auto_declare<double>("kinematics.right_arm_rpy", d.right_arm_rpy);
  auto_declare<double>("kinematics.closure_rpy", d.closure_rpy);

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration PassiveBroadcaster::command_interface_configuration() const
{
  // This is a broadcaster: it commands nothing.
  return {interface_configuration_type::NONE, {}};
}

InterfaceConfiguration PassiveBroadcaster::state_interface_configuration() const
{
  return {
    interface_configuration_type::INDIVIDUAL,
    {left_elbow_joint_ + "/position", right_elbow_joint_ + "/position"}};
}

controller_interface::CallbackReturn PassiveBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  auto node = get_node();

  left_elbow_joint_ = node->get_parameter("left_elbow_joint").as_string();
  right_elbow_joint_ = node->get_parameter("right_elbow_joint").as_string();
  passive_joints_ = node->get_parameter("passive_joints").as_string_array();

  if (passive_joints_.size() != 3) {
    RCLCPP_ERROR(
      node->get_logger(),
      "passive_joints must list exactly 3 joints [left_arm, right_arm, closure], got %zu",
      passive_joints_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  ee_marker_enable_ = node->get_parameter("ee_marker.enable").as_bool();
  ee_frame_id_ = node->get_parameter("ee_marker.frame_id").as_string();
  tool_offset_ = node->get_parameter("ee_marker.tool_offset").as_double();
  trail_max_size_ = static_cast<size_t>(node->get_parameter("ee_marker.trail_size").as_int());

  params_.L1 = node->get_parameter("kinematics.L1").as_double();
  params_.L2 = node->get_parameter("kinematics.L2").as_double();
  params_.l0 = node->get_parameter("kinematics.l0").as_double();
  params_.base_z = node->get_parameter("kinematics.base_z").as_double();
  params_.left_elbow_rpy = node->get_parameter("kinematics.left_elbow_rpy").as_double();
  params_.right_elbow_rpy = node->get_parameter("kinematics.right_elbow_rpy").as_double();
  params_.left_arm_rpy = node->get_parameter("kinematics.left_arm_rpy").as_double();
  params_.right_arm_rpy = node->get_parameter("kinematics.right_arm_rpy").as_double();
  params_.closure_rpy = node->get_parameter("kinematics.closure_rpy").as_double();

  // Full-state publisher (the single /joint_states source).
  joint_state_pub_ =
    node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", rclcpp::SystemDefaultsQoS());
  rt_joint_state_pub_ = std::make_unique<JointStatePublisher>(joint_state_pub_);

  // Pre-size the message ONCE (non-RT) so update() never allocates. Joint order:
  // [left_elbow, right_elbow, left_arm, right_arm, closure].
  {
    auto & msg = rt_joint_state_pub_->msg_;
    msg.name = {
      left_elbow_joint_, right_elbow_joint_,
      passive_joints_[0], passive_joints_[1], passive_joints_[2]};
    msg.position.assign(5, 0.0);
  }

  if (ee_marker_enable_) {
    ee_marker_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
      "~/ee_marker", rclcpp::SystemDefaultsQoS());
    ee_path_pub_ = node->create_publisher<visualization_msgs::msg::Marker>(
      "~/ee_path", rclcpp::SystemDefaultsQoS());
    // Non-RT visualization + warning at ~33 Hz.
    viz_timer_ = node->create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&PassiveBroadcaster::visualization_timer_cb, this));
  }

  RCLCPP_INFO(
    node->get_logger(),
    "PassiveBroadcaster configured: actives [%s, %s], passives [%s, %s, %s], EE marker %s",
    left_elbow_joint_.c_str(), right_elbow_joint_.c_str(),
    passive_joints_[0].c_str(), passive_joints_[1].c_str(), passive_joints_[2].c_str(),
    ee_marker_enable_ ? "enabled" : "disabled");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassiveBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Resolve state-interface indices by name so update() does no string compares.
  bool found_left = false;
  bool found_right = false;
  for (size_t i = 0; i < state_interfaces_.size(); ++i) {
    const std::string & name = state_interfaces_[i].get_name();
    if (name == left_elbow_joint_ + "/position") {
      idx_left_ = i;
      found_left = true;
    } else if (name == right_elbow_joint_ + "/position") {
      idx_right_ = i;
      found_right = true;
    }
  }
  if (!found_left || !found_right) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Could not find elbow position state interfaces (left:%d right:%d)",
      found_left, found_right);
    return controller_interface::CallbackReturn::ERROR;
  }

  have_valid_ = false;
  last_valid_ = {0.0, 0.0, 0.0, false};
  infeasible_.store(false, std::memory_order_relaxed);
  trail_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PassiveBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type PassiveBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  const std::optional<double> left = state_interfaces_[idx_left_].get_optional();
  const std::optional<double> right = state_interfaces_[idx_right_].get_optional();
  if (!left.has_value() || !right.has_value()) {
    // No fresh reading this cycle; skip without logging (RT-safe).
    return controller_interface::return_type::OK;
  }

  const double theta_left = left.value();
  const double theta_right = right.value();

  const auto pa = volcaniarm_kinematics::passiveAngles(params_, theta_left, theta_right);
  if (pa.valid) {
    last_valid_ = pa;       // cache: hold-last-valid on future infeasibility
    have_valid_ = true;
  }
  infeasible_.store(!pa.valid, std::memory_order_relaxed);

  if (!have_valid_) {
    // Never had a feasible solution yet; nothing meaningful to publish.
    return controller_interface::return_type::OK;
  }

  if (rt_joint_state_pub_ && rt_joint_state_pub_->trylock()) {
    auto & msg = rt_joint_state_pub_->msg_;
    msg.header.stamp = time;
    msg.position[0] = theta_left;
    msg.position[1] = theta_right;
    msg.position[2] = last_valid_.left_arm;
    msg.position[3] = last_valid_.right_arm;
    msg.position[4] = last_valid_.closure;
    rt_joint_state_pub_->unlockAndPublish();
  }

  // Hand the EE pose to the non-RT visualization path.
  if (ee_marker_enable_) {
    const auto pose =
      volcaniarm_kinematics::forwardEE(params_, theta_left, theta_right, tool_offset_);
    ee_pose_buffer_.writeFromNonRT(pose);
  }

  return controller_interface::return_type::OK;
}

void PassiveBroadcaster::visualization_timer_cb()
{
  auto node = get_node();

  // Throttled infeasibility warning from this non-RT context.
  if (infeasible_.load(std::memory_order_relaxed)) {
    RCLCPP_WARN_THROTTLE(
      node->get_logger(), *node->get_clock(), 1000,
      "Linkage cannot close (elbows too far apart); holding last valid passive angles");
  }

  if (!ee_marker_pub_ || !ee_path_pub_) {
    return;
  }
  const volcaniarm_kinematics::EEPose * pose = ee_pose_buffer_.readFromNonRT();
  if (pose == nullptr || !pose->valid) {
    return;
  }

  const auto stamp = node->now();

  visualization_msgs::msg::Marker sphere;
  sphere.header.frame_id = ee_frame_id_;
  sphere.header.stamp = stamp;
  sphere.ns = "ee";
  sphere.id = 0;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.action = visualization_msgs::msg::Marker::ADD;
  sphere.pose.position.x = pose->x;
  sphere.pose.position.y = pose->y;
  sphere.pose.position.z = pose->z;
  sphere.pose.orientation.w = 1.0;
  sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.05;
  sphere.color.r = 1.0f;
  sphere.color.g = 0.2f;
  sphere.color.b = 0.2f;
  sphere.color.a = 1.0f;
  ee_marker_pub_->publish(sphere);

  geometry_msgs::msg::Point pt;
  pt.x = pose->x;
  pt.y = pose->y;
  pt.z = pose->z;
  trail_.push_back(pt);
  while (trail_.size() > trail_max_size_) {
    trail_.pop_front();
  }

  visualization_msgs::msg::Marker path;
  path.header.frame_id = ee_frame_id_;
  path.header.stamp = stamp;
  path.ns = "trail";
  path.id = 1;
  path.type = visualization_msgs::msg::Marker::LINE_STRIP;
  path.action = visualization_msgs::msg::Marker::ADD;
  path.scale.x = 0.005;
  path.color.r = 0.1f;
  path.color.g = 0.6f;
  path.color.b = 1.0f;
  path.color.a = 0.9f;
  path.points.assign(trail_.begin(), trail_.end());
  ee_path_pub_->publish(path);
}

}  // namespace volcaniarm_controller

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_controller::PassiveBroadcaster,
  controller_interface::ControllerInterface)
