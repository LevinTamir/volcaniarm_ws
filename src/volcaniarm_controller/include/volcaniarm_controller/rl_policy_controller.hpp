#pragma once

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
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
  // Runs the policy on the assembled observation vector. Phase 1 stub:
  // returns zeros (so commanded target = default_joint_positions). Phase 2
  // will load an ONNX model and run real inference.
  std::vector<double> run_inference(const std::vector<double> & observation);

  // Configuration (from parameters).
  std::vector<std::string> joint_names_;
  std::vector<double> default_joint_positions_;
  double action_scale_{0.5};
  std::string model_path_;
  std::string target_topic_;
  std::string target_frame_;

  // Runtime state.
  std::vector<double> last_action_;

  // ROS I/O.
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_sub_;
  realtime_tools::RealtimeBuffer<std::shared_ptr<geometry_msgs::msg::PoseStamped>> target_buffer_;
};

}  // namespace volcaniarm_controller
