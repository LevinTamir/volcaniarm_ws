#include "volcaniarm_controllers/rl_vision_policy_controller.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <regex>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace volcaniarm_controller
{

namespace
{
constexpr const char * kImageInputName = "image";
constexpr const char * kJointPosInputName = "joint_pos_rel";
constexpr const char * kLastActionInputName = "last_action";
}  // namespace

controller_interface::CallbackReturn
RLVisionPolicyController::on_init()
{
  try {
    auto_declare<std::vector<std::string>>(
      "joints",
      std::vector<std::string>{"volcaniarm_left_elbow_joint", "volcaniarm_right_elbow_joint"});
    auto_declare<std::vector<double>>(
      "default_joint_positions", std::vector<double>{0.0, 0.0});
    auto_declare<double>("action_scale", 0.5);
    auto_declare<std::string>("model_path", "");
    auto_declare<std::string>("image_topic", "/camera/color/image_raw");
    auto_declare<int>("image_width", 96);
    auto_declare<int>("image_height", 96);
    auto_declare<std::vector<double>>("joint_position_min", std::vector<double>{});
    auto_declare<std::vector<double>>("joint_position_max", std::vector<double>{});
    auto_declare<double>("action_smoothing_alpha", 1.0);
    auto_declare<double>("image_max_age_s", 1.0);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "on_init failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLVisionPolicyController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  default_joint_positions_ = get_node()->get_parameter("default_joint_positions").as_double_array();
  action_scale_ = get_node()->get_parameter("action_scale").as_double();
  model_path_ = get_node()->get_parameter("model_path").as_string();
  image_topic_ = get_node()->get_parameter("image_topic").as_string();
  image_width_ = get_node()->get_parameter("image_width").as_int();
  image_height_ = get_node()->get_parameter("image_height").as_int();

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
  if (image_width_ <= 0 || image_height_ <= 0) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "image_width / image_height must be positive (got %ld x %ld)",
      static_cast<long>(image_width_), static_cast<long>(image_height_));
    return controller_interface::CallbackReturn::ERROR;
  }

  joint_position_min_ = get_node()->get_parameter("joint_position_min").as_double_array();
  joint_position_max_ = get_node()->get_parameter("joint_position_max").as_double_array();
  action_smoothing_alpha_ = get_node()->get_parameter("action_smoothing_alpha").as_double();
  image_max_age_s_ = get_node()->get_parameter("image_max_age_s").as_double();

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

  // Image callback runs off the RT loop: decode + cv::resize to the
  // training resolution + RGB8 and stash a uint8 NHWC buffer for the
  // RT pass to consume. cv_bridge::toCvCopy(msg, "rgb8") handles any
  // source encoding (rgb8, bgr8, bggr, mono8, etc.).
  image_sub_ = get_node()->create_subscription<sensor_msgs::msg::Image>(
    image_topic_, rclcpp::SensorDataQoS(),
    [this](const std::shared_ptr<sensor_msgs::msg::Image> msg) {
      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(*msg, "rgb8");
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "cv_bridge conversion failed: %s", e.what());
        return;
      }
      cv::Mat resized;
      if (cv_ptr->image.cols == image_width_ && cv_ptr->image.rows == image_height_) {
        resized = cv_ptr->image;
      } else {
        cv::resize(
          cv_ptr->image, resized,
          cv::Size(static_cast<int>(image_width_), static_cast<int>(image_height_)),
          0, 0, cv::INTER_AREA);
      }
      auto frame = std::make_shared<ImageFrame>();
      const size_t total = static_cast<size_t>(image_height_) *
                           static_cast<size_t>(image_width_) * 3u;
      frame->rgb_hwc.resize(total);
      // resized is contiguous CV_8UC3 after our resize/copy.
      std::memcpy(frame->rgb_hwc.data(), resized.data, total);
      frame->stamp = msg->header.stamp;
      image_frame_buffer_.writeFromNonRT(frame);
    });

  if (!model_path_.empty()) {
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
        "Failed to load bundled ONNX from '%s' — running in stub mode "
        "(arm holds at default_joint_positions).", model_path_.c_str());
    }
  } else {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "'model_path' is empty — running in stub mode (zero actions).");
  }

  RCLCPP_INFO(
    get_node()->get_logger(),
    "RLVisionPolicyController configured: %zu joints, %ldx%ld image, topic=%s, model=%s",
    joint_names_.size(), static_cast<long>(image_width_), static_cast<long>(image_height_),
    image_topic_.c_str(), model_loaded_ ? model_path_.c_str() : "<stub>");

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLVisionPolicyController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(last_action_.begin(), last_action_.end(), 0.0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
RLVisionPolicyController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
RLVisionPolicyController::command_interface_configuration() const
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
RLVisionPolicyController::state_interface_configuration() const
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
RLVisionPolicyController::update(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const size_t n = joint_names_.size();

  const auto frame_ptr = image_frame_buffer_.readFromRT();
  const bool has_frame = frame_ptr != nullptr && *frame_ptr != nullptr;

  // Stale-image watchdog: if the latest frame is older than
  // image_max_age_s (publisher dropped, network glitch), fall back
  // to the home pose. Skipped when stamp is zero — that's the same
  // policy the state-based controller uses for its target topic, so
  // simple test publishers without stamps still work.
  bool frame_stale = false;
  if (has_frame) {
    const rclcpp::Time stamp = (*frame_ptr)->stamp;
    if (stamp.nanoseconds() > 0) {
      const double age = (get_node()->get_clock()->now() - stamp).seconds();
      frame_stale = (age > image_max_age_s_);
    }
  }

  if (!has_frame || frame_stale) {
    for (size_t i = 0; i < n; ++i) {
      if (!command_interfaces_[i].set_value(default_joint_positions_[i])) {
        RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Failed to write command on %s", command_interfaces_[i].get_name().c_str());
      }
    }
    std::fill(last_action_.begin(), last_action_.end(), 0.0);
    if (frame_stale) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Image stale (> %.2f s); holding default pose", image_max_age_s_);
    }
    return controller_interface::return_type::OK;
  }

  // joint_pos_rel = q − default
  std::vector<double> joint_pos_rel(n);
  for (size_t i = 0; i < n; ++i) {
    const auto q_opt = state_interfaces_[i].get_optional();
    if (!q_opt.has_value()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "State interface %s has no value", state_interfaces_[i].get_name().c_str());
      return controller_interface::return_type::ERROR;
    }
    joint_pos_rel[i] = q_opt.value() - default_joint_positions_[i];
  }

  auto raw_action = run_inference((*frame_ptr)->rgb_hwc, joint_pos_rel);
  if (raw_action.size() != n) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "Inference returned %zu values, expected %zu", raw_action.size(), n);
    return controller_interface::return_type::ERROR;
  }

  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(raw_action[i])) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Inference produced non-finite output; commanding default pose");
      std::fill(raw_action.begin(), raw_action.end(), 0.0);
      break;
    }
  }

  if (action_smoothing_alpha_ < 1.0) {
    for (size_t i = 0; i < n; ++i) {
      raw_action[i] = action_smoothing_alpha_ * raw_action[i]
                    + (1.0 - action_smoothing_alpha_) * last_action_[i];
    }
  }

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
RLVisionPolicyController::load_model()
{
  try {
    onnx_env_ = std::make_unique<Ort::Env>(ORT_LOGGING_LEVEL_WARNING, "RLVisionPolicyController");
    onnx_memory_info_ = std::make_unique<Ort::MemoryInfo>(
      Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault));

    Ort::SessionOptions opts;
    opts.SetIntraOpNumThreads(1);
    opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
    onnx_session_ = std::make_unique<Ort::Session>(*onnx_env_, model_path_.c_str(), opts);

    Ort::AllocatorWithDefaultOptions allocator;
    const size_t n_inputs = onnx_session_->GetInputCount();
    const size_t n_outputs = onnx_session_->GetOutputCount();
    if (n_inputs != 3 || n_outputs != 1) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Bundled ONNX must have 3 inputs (image, joint_pos_rel, last_action) and 1 output. "
        "Got %zu / %zu — exporter mismatch?", n_inputs, n_outputs);
      return false;
    }

    input_names_owned_.resize(3);
    input_name_ptrs_.resize(3);
    image_input_idx_ = -1;
    joint_pos_input_idx_ = -1;
    last_action_input_idx_ = -1;
    for (size_t i = 0; i < 3; ++i) {
      input_names_owned_[i] = onnx_session_->GetInputNameAllocated(i, allocator).get();
      input_name_ptrs_[i] = input_names_owned_[i].c_str();
      if (input_names_owned_[i] == kImageInputName) {
        image_input_idx_ = static_cast<int>(i);
      } else if (input_names_owned_[i] == kJointPosInputName) {
        joint_pos_input_idx_ = static_cast<int>(i);
      } else if (input_names_owned_[i] == kLastActionInputName) {
        last_action_input_idx_ = static_cast<int>(i);
      }
    }
    if (image_input_idx_ < 0 || joint_pos_input_idx_ < 0 || last_action_input_idx_ < 0) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Bundled ONNX inputs must be named [\"image\", \"joint_pos_rel\", \"last_action\"]. "
        "Got [%s, %s, %s].",
        input_names_owned_[0].c_str(), input_names_owned_[1].c_str(),
        input_names_owned_[2].c_str());
      return false;
    }

    output_names_owned_.emplace_back(onnx_session_->GetOutputNameAllocated(0, allocator).get());
    output_name_ptrs_ = {output_names_owned_[0].c_str()};

    model_loaded_ = true;
    RCLCPP_INFO(
      get_node()->get_logger(),
      "Loaded bundled ONNX: image='%s', joint_pos_rel='%s', last_action='%s', output='%s'",
      input_name_ptrs_[image_input_idx_], input_name_ptrs_[joint_pos_input_idx_],
      input_name_ptrs_[last_action_input_idx_], output_name_ptrs_[0]);
    return true;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "ONNX load failed: %s", e.what());
    model_loaded_ = false;
    return false;
  }
}

std::vector<double>
RLVisionPolicyController::run_inference(
  const std::vector<uint8_t> & image_buffer,
  const std::vector<double> & joint_pos_rel)
{
  const size_t n = joint_names_.size();
  if (!model_loaded_) {
    return std::vector<double>(n, 0.0);
  }

  // Expected shapes of the bundled ONNX exported by
  //   source/volcaniarm/scripts/rsl_rl/export_onnx_bundle.py
  //   image          uint8  (1, H, W, 3)
  //   joint_pos_rel  float  (1, N)
  //   last_action    float  (1, N)
  const std::array<int64_t, 4> image_shape{
    1, image_height_, image_width_, 3};
  const std::array<int64_t, 2> proprio_shape{
    1, static_cast<int64_t>(n)};

  // ONNX RunOptions order must match input_name_ptrs_, not our index
  // labels — populate value/name arrays in lockstep with index 0..2.
  std::array<const char *, 3> ordered_input_names{};
  std::array<Ort::Value, 3> ordered_input_values{
    Ort::Value{nullptr}, Ort::Value{nullptr}, Ort::Value{nullptr}};

  // Build a writable copy of the image buffer (Ort takes a non-const
  // pointer for tensor-from-buffer construction).
  std::vector<uint8_t> image_copy = image_buffer;
  std::vector<float> joint_pos_rel_f(joint_pos_rel.begin(), joint_pos_rel.end());
  std::vector<float> last_action_f(last_action_.begin(), last_action_.end());

  try {
    ordered_input_names[image_input_idx_] = input_name_ptrs_[image_input_idx_];
    ordered_input_values[image_input_idx_] = Ort::Value::CreateTensor<uint8_t>(
      *onnx_memory_info_, image_copy.data(), image_copy.size(),
      image_shape.data(), image_shape.size());

    ordered_input_names[joint_pos_input_idx_] = input_name_ptrs_[joint_pos_input_idx_];
    ordered_input_values[joint_pos_input_idx_] = Ort::Value::CreateTensor<float>(
      *onnx_memory_info_, joint_pos_rel_f.data(), joint_pos_rel_f.size(),
      proprio_shape.data(), proprio_shape.size());

    ordered_input_names[last_action_input_idx_] = input_name_ptrs_[last_action_input_idx_];
    ordered_input_values[last_action_input_idx_] = Ort::Value::CreateTensor<float>(
      *onnx_memory_info_, last_action_f.data(), last_action_f.size(),
      proprio_shape.data(), proprio_shape.size());

    auto output_tensors = onnx_session_->Run(
      Ort::RunOptions{nullptr},
      ordered_input_names.data(), ordered_input_values.data(), 3,
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
  volcaniarm_controller::RLVisionPolicyController,
  controller_interface::ControllerInterface)
