#include "volcaniarm_controller/rl_policy_controller.hpp"

#include <algorithm>
#include <string>
#include <vector>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace volcaniarm_controller
{

controller_interface::CallbackReturn
RLPolicyController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints",
      std::vector<std::string>{"volcaniarm_left_elbow_joint", "volcaniarm_right_elbow_joint"});
    auto_declare<std::vector<double>>(
      "default_joint_positions", std::vector<double>{0.0, 0.0});
    auto_declare<double>("action_scale", 0.5);
    auto_declare<std::string>("model_path", "");
    auto_declare<std::string>("target_topic", "/ee_target_pose");
    auto_declare<std::string>("target_frame", "volcaniarm_base_link");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLPolicyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  default_joint_positions_ = get_node()->get_parameter("default_joint_positions").as_double_array();
  action_scale_ = get_node()->get_parameter("action_scale").as_double();
  model_path_ = get_node()->get_parameter("model_path").as_string();
  target_topic_ = get_node()->get_parameter("target_topic").as_string();
  target_frame_ = get_node()->get_parameter("target_frame").as_string();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter is empty");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (default_joint_positions_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "default_joint_positions size (%zu) must match joints size (%zu)",
      default_joint_positions_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  last_action_.assign(joint_names_.size(), 0.0);

  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      target_buffer_.writeFromNonRT(msg);
    });

  RCLCPP_INFO(
    get_node()->get_logger(),
    "RLPolicyController configured: %zu joints, action_scale=%.3f, target_topic=%s, model=%s",
    joint_names_.size(), action_scale_, target_topic_.c_str(),
    model_path_.empty() ? "<stub>" : model_path_.c_str());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLPolicyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(last_action_.begin(), last_action_.end(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLPolicyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RLPolicyController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/position");
  }
  return cfg;
}

controller_interface::InterfaceConfiguration
RLPolicyController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration cfg;
  cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  cfg.names.reserve(joint_names_.size());
  for (const auto & name : joint_names_) {
    cfg.names.push_back(name + "/position");
  }
  return cfg;
}

controller_interface::return_type
RLPolicyController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto target_ptr = target_buffer_.readFromRT();
  const bool has_target = target_ptr != nullptr && *target_ptr != nullptr;

  // Observation vector matching Volcaniarm-Reach-v0 obs order:
  //   [joint_pos_rel (N), pose_command (7), last_action (N)]
  const size_t n = joint_names_.size();
  std::vector<double> observation;
  observation.reserve(2 * n + 7);

  // joint_pos_rel = q - q_default
  for (size_t i = 0; i < n; ++i) {
    const auto q_opt = state_interfaces_[i].get_optional();
    if (!q_opt.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "State interface %s has no value", state_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
    observation.push_back(q_opt.value() - default_joint_positions_[i]);
  }

  // pose_command: [x, y, z, qw, qx, qy, qz] in target_frame
  if (has_target) {
    const auto & p = (*target_ptr)->pose.position;
    const auto & o = (*target_ptr)->pose.orientation;
    observation.push_back(p.x);
    observation.push_back(p.y);
    observation.push_back(p.z);
    observation.push_back(o.w);
    observation.push_back(o.x);
    observation.push_back(o.y);
    observation.push_back(o.z);
  } else {
    // No target yet — zero pose (identity quat). Policy will effectively idle.
    observation.insert(observation.end(), {0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0});
  }

  // last_action (raw, pre-scale)
  for (const double a : last_action_) {
    observation.push_back(a);
  }

  const auto raw_action = run_inference(observation);
  if (raw_action.size() != n) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Inference returned %zu values, expected %zu", raw_action.size(), n);
    return controller_interface::return_type::ERROR;
  }

  // target_q = default + scale * raw_action (matches training's
  // JointPositionAction with use_default_offset=True, scale=0.5).
  for (size_t i = 0; i < n; ++i) {
    const double target_q = default_joint_positions_[i] + action_scale_ * raw_action[i];
    if (!command_interfaces_[i].set_value(target_q)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to write command on %s", command_interfaces_[i].get_name().c_str());
    }
  }
  last_action_ = raw_action;

  return controller_interface::return_type::OK;
}

std::vector<double>
RLPolicyController::run_inference(const std::vector<double> & /*observation*/)
{
  // Phase 1 stub: return zeros so commanded target = default_joint_positions.
  // Phase 2 will load an ONNX model and run real inference here.
  return std::vector<double>(joint_names_.size(), 0.0);
}

}  // namespace volcaniarm_controller

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_controller::RLPolicyController,
  controller_interface::ControllerInterface)
