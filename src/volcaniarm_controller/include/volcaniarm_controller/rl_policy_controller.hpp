#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "onnxruntime_cxx_api.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"

namespace volcaniarm_controller
{

// RL policy controller for the volcaniarm 5-bar planar arm.
//
// Consumes an EE target pose in `volcaniarm_base_link` frame, reads
// actuated elbow joint positions from the hardware, runs an ONNX policy
// (trained in Isaac Lab), and writes position targets back to the two
// elbow joints. Matches the Volcaniarm-Reach-v0 training contract.
class RLPolicyController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Loads the ONNX policy from `model_path_`. Returns true on success.
  // If `model_path_` is empty or load fails, the controller keeps running
  // with zero actions (arm holds at default) — useful for wiring tests.
  bool load_model();

  // Runs the policy on the observation. If the model isn't loaded,
  // returns zeros (so commanded target = default_joint_positions).
  std::vector<double> run_inference(const std::vector<double> & observation);

  // Configuration (from parameters).
  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_positions_;
  double action_scale_{0.5};
  std::string model_path_;
  std::string target_topic_;
  std::string target_frame_;

  // Safety / polish parameters. Empty / unity defaults preserve the
  // original behaviour; the operator opts in via YAML.
  //   joint_position_{min,max}: hard per-joint clamps applied AFTER
  //     the default-offset + scale step. Empty = no clamp.
  //   action_smoothing_alpha: EMA over raw policy outputs.
  //     1.0 = no smoothing (default), 0.0 = freeze action.
  //   target_max_age_s: if the latest target message stamp is older
  //     than this (and was non-zero), the controller falls back to
  //     holding default_joint_positions until a fresh target arrives.
  std::vector<double> joint_position_min_;
  std::vector<double> joint_position_max_;
  double action_smoothing_alpha_{1.0};
  double target_max_age_s_{1.0};

  // Runtime state.
  std::vector<double> last_action_;

  // ONNX runtime state.
  std::unique_ptr<Ort::Env> onnx_env_;
  std::unique_ptr<Ort::MemoryInfo> onnx_memory_info_;
  std::unique_ptr<Ort::Session> onnx_session_;
  std::vector<std::string> input_names_owned_;
  std::vector<std::string> output_names_owned_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;
  std::vector<int64_t> input_shape_;
  bool model_loaded_{false};

  // ROS I/O.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> target_buffer_;
};

}  // namespace volcaniarm_controller
