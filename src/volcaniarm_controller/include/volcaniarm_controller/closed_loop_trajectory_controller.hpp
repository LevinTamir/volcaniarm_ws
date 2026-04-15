#ifndef VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_
#define VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace volcaniarm_controller
{

class ClosedLoopTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ClosedLoopTrajectoryController() = default;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void elbow_tip(double elbow_rpy, double theta, double shoulder_y,
                 double & y_out, double & z_out) const;

  bool compute_ee(double y_l, double z_l, double y_r, double z_r,
                  double & y_ee, double & z_ee) const;

  bool compute_passive_angles(
    double theta_left, double theta_right,
    double & left_arm_out, double & right_arm_out, double & closure_out) const;

  // Compute EE position in volcaniarm_base_link frame
  void compute_ee_position(double theta_left, double theta_right,
                           double & x_ee, double & y_ee, double & z_ee) const;

  void publish_ee_markers(const rclcpp::Time & time,
                          double x, double y, double z);

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr passive_joint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr ee_path_pub_;

  // Configuration from YAML
  std::vector<std::string> passive_joint_names_;
  std::string left_elbow_joint_name_;
  std::string right_elbow_joint_name_;

  // EE visualization
  bool publish_ee_marker_{true};
  std::string ee_frame_id_{"volcaniarm_base_link"};
  double tool_offset_{0.0};  // X offset for planar mechanism
  size_t trail_max_size_{300};
  std::deque<geometry_msgs::msg::Point> trail_;

  // Kinematic parameters
  double L1_{0.41621};
  double L2_{0.65};
  double l0_{0.215};
  double base_z_{0.0632};

  // URDF joint frame Rx offsets
  double left_elbow_rpy_{0.7854};
  double right_elbow_rpy_{-0.7854};
  double left_arm_rpy_{-1.7053};
  double right_arm_rpy_{1.7053};
  double closure_rpy_{1.8398};
};

}  // namespace volcaniarm_controller

#endif  // VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_
