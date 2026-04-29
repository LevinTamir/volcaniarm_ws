#include "volcaniarm_controller/rl_policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <regex>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
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
    // Safety polish (see header for semantics). Empty / unity defaults
    // preserve original controller behaviour.
    auto_declare<std::vector<double>>("joint_position_min", std::vector<double>{});
    auto_declare<std::vector<double>>("joint_position_max", std::vector<double>{});
    auto_declare<double>("action_smoothing_alpha", 1.0);
    auto_declare<double>("target_max_age_s", 1.0);
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

  joint_position_min_ = get_node()->get_parameter("joint_position_min").as_double_array();
  joint_position_max_ = get_node()->get_parameter("joint_position_max").as_double_array();
  action_smoothing_alpha_ = get_node()->get_parameter("action_smoothing_alpha").as_double();
  target_max_age_s_ = get_node()->get_parameter("target_max_age_s").as_double();

  // Per-joint clamps must be either empty (clamping off) or sized to
  // match the joint count. A mismatched length is a config error
  // worth refusing rather than silently clamping the wrong axes.
  if (!joint_position_min_.empty() && joint_position_min_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "joint_position_min size (%zu) must be empty or match joints size (%zu)",
      joint_position_min_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (!joint_position_max_.empty() && joint_position_max_.size() != joint_names_.size()) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "joint_position_max size (%zu) must be empty or match joints size (%zu)",
      joint_position_max_.size(), joint_names_.size());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (action_smoothing_alpha_ < 0.0 || action_smoothing_alpha_ > 1.0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "action_smoothing_alpha must be in [0, 1] (got %.3f)", action_smoothing_alpha_);
    return controller_interface::CallbackReturn::ERROR;
  }

  last_action_.assign(joint_names_.size(), 0.0);

  target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    target_topic_, rclcpp::SystemDefaultsQoS(),
    [this](const std::shared_ptr<geometry_msgs::msg::PoseStamped> msg) {
      target_buffer_.writeFromNonRT(msg);
    });

  if (!model_path_.empty()) {
    // Expand $(find-pkg-share <pkg>) substitutions so configs can be
    // deployment-agnostic (matching ros2_control_demos example 18 idiom).
    static const std::regex pkg_share_regex(R"(\$\(find-pkg-share\s+([^\)]+)\))");
    std::smatch match;
    while (std::regex_search(model_path_, match, pkg_share_regex)) {
      try {
        const std::string share = ament_index_cpp::get_package_share_directory(match[1].str());
        model_path_.replace(match.position(0), match.length(0), share);
      } catch (const std::exception & e) {
        RCLCPP_ERROR(
          get_node()->get_logger(),
          "Cannot resolve $(find-pkg-share %s): %s", match[1].str().c_str(), e.what());
        return controller_interface::CallbackReturn::ERROR;
      }
    }

    if (!load_model()) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Failed to load ONNX model from '%s' — running in stub mode. "
        "Drop a trained policy.onnx there and reload the controller.",
        model_path_.c_str());
    }
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "'model_path' is empty — running in stub mode (zero actions). "
      "Set model_path to a trained policy.onnx to enable inference.");
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "RLPolicyController configured: %zu joints, action_scale=%.3f, target_topic=%s, model=%s",
    joint_names_.size(), action_scale_, target_topic_.c_str(),
    model_loaded_ ? model_path_.c_str() : "<stub>");

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
  const size_t n = joint_names_.size();

  const auto target_ptr = target_buffer_.readFromRT();
  const bool has_target = target_ptr != nullptr && *target_ptr != nullptr;

  // Watchdog: if the latest target is too old (e.g. publisher died,
  // network glitch), fall back to default. Skipped when the message
  // stamp is zero -- that means the publisher didn't stamp at all,
  // and we don't want a degraded watchdog blocking everything.
  bool target_stale = false;
  if (has_target) {
    const rclcpp::Time stamp = (*target_ptr)->header.stamp;
    if (stamp.nanoseconds() > 0) {
      const double age = (get_node()->get_clock()->now() - stamp).seconds();
      target_stale = (age > target_max_age_s_);
    }
  }

  // No target message yet (or stale) → hold the default (home) pose
  // instead of feeding stale / zero targets into the policy. The
  // policy was trained on a specific target distribution; feeding
  // out-of-distribution poses produces unpredictable joint commands.
  if (!has_target || target_stale) {
    for (size_t i = 0; i < n; ++i) {
      if (!command_interfaces_[i].set_value(default_joint_positions_[i])) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Failed to write command on %s", command_interfaces_[i].get_name().c_str());
      }
    }
    std::fill(last_action_.begin(), last_action_.end(), 0.0);
    if (target_stale) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Target stale (> %.2f s); holding default pose", target_max_age_s_);
    }
    return controller_interface::return_type::OK;
  }

  // Observation vector matching Volcaniarm-Reach-v0 obs order:
  //   [joint_pos_rel (N), pose_command (7), last_action (N)]
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
  const auto & p = (*target_ptr)->pose.position;
  const auto & o = (*target_ptr)->pose.orientation;
  observation.push_back(p.x);
  observation.push_back(p.y);
  observation.push_back(p.z);
  observation.push_back(o.w);
  observation.push_back(o.x);
  observation.push_back(o.y);
  observation.push_back(o.z);

  // last_action (raw, pre-scale)
  for (const double a : last_action_) {
    observation.push_back(a);
  }

  auto raw_action = run_inference(observation);
  if (raw_action.size() != n) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Inference returned %zu values, expected %zu", raw_action.size(), n);
    return controller_interface::return_type::ERROR;
  }

  // NaN guard: a corrupt model or a numerical blow-up shouldn't get
  // commanded to the joints. If any value is non-finite we treat the
  // whole step as zero action (= command default) and warn.
  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(raw_action[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Inference produced non-finite output; commanding default pose");
      std::fill(raw_action.begin(), raw_action.end(), 0.0);
      break;
    }
  }

  // EMA action smoothing: a = alpha*new + (1-alpha)*last. alpha=1.0
  // (default) is the original no-smoothing behaviour. Lower values
  // damp oscillation but lag the policy. The smoothed action also
  // feeds back into the obs vector next tick (last_action_), keeping
  // the policy view consistent with what's actually commanded.
  if (action_smoothing_alpha_ < 1.0) {
    for (size_t i = 0; i < n; ++i) {
      raw_action[i] = action_smoothing_alpha_ * raw_action[i]
                    + (1.0 - action_smoothing_alpha_) * last_action_[i];
    }
  }

  // target_q = default + scale * raw_action (matches training's
  // JointPositionAction with use_default_offset=True, scale=0.5),
  // then clamped to per-joint safety limits if configured.
  for (size_t i = 0; i < n; ++i) {
    double target_q = default_joint_positions_[i] + action_scale_ * raw_action[i];
    if (!joint_position_min_.empty() && !joint_position_max_.empty()) {
      target_q = std::clamp(target_q, joint_position_min_[i], joint_position_max_[i]);
    }
    if (!command_interfaces_[i].set_value(target_q)) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Failed to write command on %s", command_interfaces_[i].get_name().c_str());
    }
  }
  last_action_ = raw_action;

  return controller_interface::return_type::OK;
}

bool
RLPolicyController::load_model()
{
  try {
    onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "RLPolicyController");
    onnx_memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, model_path_.c_str(), opts);

    Ort::AllocatorWithDefaultOptions allocator;
    const size_t n_inputs = onnx_session_->GetInputCount();
    const size_t n_outputs = onnx_session_->GetOutputCount();
    if (n_inputs != 1 || n_outputs != 1) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "ONNX model must have exactly 1 input and 1 output (got %zu / %zu)",
        n_inputs, n_outputs);
      return false;
    }

    input_names_owned_.emplace_back(onnx_session_->GetInputNameAllocated(0, allocator).get());
    output_names_owned_.emplace_back(onnx_session_->GetOutputNameAllocated(0, allocator).get());
    input_name_ptrs_ = {input_names_owned_[0].c_str()};
    output_name_ptrs_ = {output_names_owned_[0].c_str()};

    // Resolve any dynamic dim (-1) to 1. rsl_rl's exported policy expects
    // shape [1, obs_dim], but some exporters leave batch as dynamic.
    auto type_info = onnx_session_->GetInputTypeInfo(0);
    auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
    input_shape_ = tensor_info.GetShape();
    for (auto & d : input_shape_) {
      if (d < 0) {
        d = 1;
      }
    }

    model_loaded_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(), "Loaded ONNX policy: input=%s, output=%s",
      input_name_ptrs_[0], output_name_ptrs_[0]);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "ONNX load failed: %s", e.what());
    model_loaded_ = false;
    return false;
  }
}

std::vector<double>
RLPolicyController::run_inference(const std::vector<double> & observation)
{
  const size_t n = joint_names_.size();
  if (!model_loaded_) {
    return std::vector<double>(n, 0.0);
  }

  try {
    std::vector<float> input(observation.begin(), observation.end());
    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
      *onnx_memory_info_, input.data(), input.size(),
      input_shape_.data(), input_shape_.size());

    auto output_tensors = onnx_session_->Run(
      Ort::RunOptions{nullptr},
      input_name_ptrs_.data(), &input_tensor, 1,
      output_name_ptrs_.data(), 1);

    const float * out_data = output_tensors.front().GetTensorData<float>();
    const auto out_shape = output_tensors.front().GetTensorTypeAndShapeInfo().GetShape();
    size_t out_count = 1;
    for (const auto d : out_shape) {
      out_count *= (d > 0 ? static_cast<size_t>(d) : 1u);
    }
    if (out_count != n) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "ONNX output count %zu != joint count %zu", out_count, n);
      return std::vector<double>(n, 0.0);
    }
    std::vector<double> out(n);
    for (size_t i = 0; i < n; ++i) {
      out[i] = static_cast<double>(out_data[i]);
    }
    return out;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "ONNX inference failed: %s", e.what());
    return std::vector<double>(n, 0.0);
  }
}

}  // namespace volcaniarm_controller

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_controller::RLPolicyController,
  controller_interface::ControllerInterface)
