#ifndef VOLCANIARM_CONTROLLERS__PASSIVE_BROADCASTER_HPP_
#define VOLCANIARM_CONTROLLERS__PASSIVE_BROADCASTER_HPP_

#include <atomic>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "volcaniarm_kinematics/five_bar.hpp"
#include "volcaniarm_kinematics/params.hpp"

namespace volcaniarm_controller
{

// Controller-interface plugin that claims only the two actuated elbow position
// STATE interfaces, solves the three passive five-bar joint angles via
// volcaniarm_kinematics, and publishes all five joints as a single
// sensor_msgs/JointState (one timestamp) through a RealtimePublisher. The EE
// marker/trail visualization and the closure-infeasibility warning run on a
// non-realtime timer so update() stays allocation-, log- and publish-light.
class PassiveBroadcaster : public controller_interface::ControllerInterface
{
public:
  PassiveBroadcaster() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Non-RT timer callback: publishes EE markers + the throttled warning.
  void visualization_timer_cb();

  // ── Configuration ──────────────────────────────────────────────
  std::string left_elbow_joint_;
  std::string right_elbow_joint_;
  std::vector<std::string> passive_joints_;   // [left_arm, right_arm, closure]
  volcaniarm_kinematics::Params params_;

  bool ee_marker_enable_{true};
  std::string ee_frame_id_{"volcaniarm_base_link"};
  double tool_offset_{0.0};
  size_t trail_max_size_{300};

  // State-interface indices resolved once in on_activate().
  size_t idx_left_{0};
  size_t idx_right_{0};

  // ── Realtime publishers ────────────────────────────────────────
  using JointStatePublisher = realtime_tools::RealtimePublisher<sensor_msgs::msg::JointState>;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::JointState>> joint_state_pub_;
  std::unique_ptr<JointStatePublisher> rt_joint_state_pub_;

  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> ee_marker_pub_;
  std::shared_ptr<rclcpp::Publisher<visualization_msgs::msg::Marker>> ee_path_pub_;

  // ── RT <-> non-RT boundary ─────────────────────────────────────
  realtime_tools::RealtimeBuffer<volcaniarm_kinematics::EEPose> ee_pose_buffer_;
  std::atomic<bool> infeasible_{false};

  // ── RT-local state (touched only inside update()) ──────────────
  volcaniarm_kinematics::PassiveAngles last_valid_{0.0, 0.0, 0.0, false};
  bool have_valid_{false};

  // ── Non-RT-local state (touched only inside the timer) ─────────
  std::deque<geometry_msgs::msg::Point> trail_;
  rclcpp::TimerBase::SharedPtr viz_timer_;
};

}  // namespace volcaniarm_controller

#endif  // VOLCANIARM_CONTROLLERS__PASSIVE_BROADCASTER_HPP_
