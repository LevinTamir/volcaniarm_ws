#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "onnxruntime_cxx_api.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace volcaniarm_controller
{

// Vision RL policy controller for the volcaniarm 5-bar planar arm.
//
// Sibling of `RLPolicyController`. Instead of consuming an EE target
// pose topic, this controller subscribes to a camera image and feeds
// the bundled (ResNet18 + actor) ONNX exported by IsaacLab's
// `export_onnx_bundle.py` for the `Volcaniarm-Reach-Vision-v0` task.
// The ONNX has three named inputs:
//   image           uint8  (1, H, W, 3)   raw RGB frame at training resolution
//   joint_pos_rel   float  (1, N)         q − default
//   last_action     float  (1, N)         previous raw action (pre-scale)
// and one output:
//   action          float  (1, N)
//
// The image callback runs off the RT loop (cv_bridge decode + cv::resize
// to the training resolution), and stashes the resized uint8 buffer
// into a realtime-safe queue. The RT loop reads the latest buffer,
// builds the three input tensors, runs inference, and writes joint
// position targets — same downstream pipeline as the state-based
// controller (action_scale, EMA smoothing, NaN guard, joint clamps).
//
// REST/WORK mode: the controller ACTIVATES IN REST — it ramps from the
// current pose to default_joint_positions over rest_ramp_s and holds
// there, skipping image processing and ONNX inference entirely. Policy
// control starts only after `~/set_work_mode` (std_srvs/SetBool) is
// called with data=true; data=false ramps back home. Every
// (re)activation resets the mode to the `start_in_work_mode` parameter
// (default false) — a service call made while inactive does not carry
// over. The current mode is latched on `~/work_mode` (std_msgs/Bool).
class RLVisionPolicyController : public controller_interface::ControllerInterface
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
  bool load_model();

  // Runs the bundled ONNX on the latest decoded image + proprio state.
  // Returns the action vector (size = num joints). On failure (model
  // not loaded, shape mismatch, ONNX exception) returns a zero vector
  // so the arm holds at default.
  std::vector<double> run_inference(
    const std::vector<uint8_t> & image_buffer,
    const std::vector<double> & joint_pos_rel);

  // One frame's worth of pre-processed image data + the timestamp it
  // was sent. The decode/resize happens in the topic callback (off
  // the RT loop) so the update() pass only does an O(1) read.
  struct ImageFrame
  {
    std::vector<uint8_t> rgb_hwc;  // size = height * width * 3
    rclcpp::Time stamp;
  };

  // Configuration (from parameters).
  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_positions_;
  double action_scale_{0.5};
  std::string model_path_;
  std::string image_topic_;
  int64_t image_width_{96};
  int64_t image_height_{96};

  // Safety / polish parameters (see header on RLPolicyController for
  // semantics; image_max_age_s replaces target_max_age_s).
  std::vector<double> joint_position_min_;
  std::vector<double> joint_position_max_;
  double action_smoothing_alpha_{1.0};
  double image_max_age_s_{1.0};

  // REST/WORK mode. `work_mode_` is written by the ~/set_work_mode
  // service callback (non-RT executor thread) and read once per
  // update() tick — a single lock-free word, no buffer needed.
  std::atomic<bool> work_mode_{false};
  bool start_in_work_mode_{false};  // initial mode on every activation
  double rest_ramp_s_{1.0};         // current pose -> home ramp duration

  // REST-ramp state, touched only from the RT thread (update /
  // on_activate) — no synchronization needed.
  bool prev_work_mode_{false};
  bool rest_entry_pending_{true};   // force ramp capture on first REST tick
  std::vector<double> ramp_start_positions_;
  double ramp_elapsed_s_{0.0};

  // Runtime state.
  std::vector<double> last_action_;

  // ONNX runtime state. The bundled model has 3 inputs / 1 output;
  // we resolve their names from the session at load time.
  std::unique_ptr<Ort::Env> onnx_env_;
  std::unique_ptr<Ort::MemoryInfo> onnx_memory_info_;
  std::unique_ptr<Ort::Session> onnx_session_;
  std::vector<std::string> input_names_owned_;
  std::vector<std::string> output_names_owned_;
  std::vector<const char *> input_name_ptrs_;
  std::vector<const char *> output_name_ptrs_;
  // Indexes into input_name_ptrs_ for each named input. Resolved by
  // matching the ONNX graph's input names to the canonical names used
  // by export_onnx_bundle.py.
  int image_input_idx_{-1};
  int joint_pos_input_idx_{-1};
  int last_action_input_idx_{-1};
  bool model_loaded_{false};

  // ROS I/O.
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<ImageFrame>> image_frame_buffer_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr set_work_mode_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mode_pub_;

  // Publish the current mode on ~/work_mode (latched). Non-RT contexts
  // only (on_activate, service callback) — never call from update().
  void publish_mode(bool work);
};

}  // namespace volcaniarm_controller
