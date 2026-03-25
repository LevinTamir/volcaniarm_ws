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
  // Compute elbow tip position in volcaniarm_base_link frame
  void elbow_tip(double elbow_rpy, double theta, double shoulder_y,
                 double & y_out, double & z_out) const;

  // Compute end-effector via circle-circle intersection
  bool compute_ee(double y_l, double z_l, double y_r, double z_r,
                  double & y_ee, double & z_ee) const;

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
  double L1_{0.41621};    // Upper link length (elbow link)
  double L2_{0.65};       // Lower link length (arm link)
  double l0_{0.215};      // Shoulder separation from center
  double base_z_{0.0632}; // Elbow joint Z offset in volcaniarm_base_link

  // URDF joint frame Rx offsets
  double left_elbow_rpy_{0.7854};
  double right_elbow_rpy_{-0.7854};
  double left_arm_rpy_{-1.7053};
  double right_arm_rpy_{1.7053};
  double closure_rpy_{1.8398};
};

}  // namespace volcaniarm_controller

#endif  // VOLCANIARM_CONTROLLER__CLOSED_LOOP_TRAJECTORY_CONTROLLER_HPP_
