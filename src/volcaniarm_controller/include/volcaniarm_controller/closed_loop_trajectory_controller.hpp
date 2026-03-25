#ifndef VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_
#define VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
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
  // FK: compute end-effector (y, z) from elbow angles
  std::pair<double, double> fk(double theta_right, double theta_left) const;

  // Compute passive arm and closure joint angles
  void compute_passive_angles(
    double theta_left, double theta_right,
    double & left_arm_out, double & right_arm_out, double & closure_out) const;

  // Publisher for passive joint states
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr passive_joint_pub_;

  // Configuration from YAML
  std::vector<std::string> passive_joint_names_;
  std::string left_elbow_joint_name_;
  std::string right_elbow_joint_name_;

  // Kinematic parameters
  double L1_{0.41621};   // Upper link length (elbow link)
  double L2_{0.65};      // Lower link length (arm link)
  double l0_{0.215};     // Shoulder separation from center
  double base_z_{0.0582}; // Base height offset

  // URDF joint frame Rx offsets (cumulative rotation constants)
  // cum_left_base  = pi + left_elbow_rpy + left_arm_rpy  = 2.2217
  // cum_right_base = pi + right_elbow_rpy + right_arm_rpy = 4.0615
  double cum_left_base_{2.2217};
  double cum_right_base_{4.0615};
  double closure_rpy_{1.8398};

  // Home-position calibration offsets (computed once at configure)
  double left_arm_home_offset_{0.0};
  double right_arm_home_offset_{0.0};
};

}  // namespace volcaniarm_controller

#endif  // VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_
