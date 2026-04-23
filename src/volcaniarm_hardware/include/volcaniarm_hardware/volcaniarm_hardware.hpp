#pragma once

#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"

namespace volcaniarm_hardware
{

class VolcaniArmHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VolcaniArmHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Serial config
  std::string port_;
  int baud_{115200};
  double steps_per_rev_{1600.0};

  int fd_{-1};  // serial file descriptor

  // Right elbow joint state + command
  double hw_position_right_elbow_{0.0};
  double hw_velocity_right_elbow_{0.0};
  double hw_position_command_right_elbow_{0.0};

  // Left elbow joint state + command
  double hw_position_left_elbow_{0.0};
  double hw_velocity_left_elbow_{0.0};
  double hw_position_command_left_elbow_{0.0};

  // Home offsets: URDF_position = physical_position + home_offset
  double right_elbow_home_offset_{0.0};
  double left_elbow_home_offset_{0.0};

  // Whether to trigger limit-switch homing automatically in on_configure.
  // When false, the service "volcaniarm_hardware/home" can still be used
  // to home manually at any time.
  bool auto_home_on_configure_{false};

  // Serial read buffer for parsing ESP responses
  static const int SERIAL_BUF_SIZE = 256;
  char serial_buf_[SERIAL_BUF_SIZE];
  int serial_buf_pos_{0};

  // Previous position for velocity calculation
  double prev_position_right_elbow_{0.0};
  double prev_position_left_elbow_{0.0};

  bool configure_port_();
  bool send_position_command_rad_(double position_rad_right, double position_rad_left);
  void read_serial_();

  /// Run homing sequence. Currently sends 'H' to ESP which resets to 0,0.
  /// TODO: replace with limit switch homing.
  bool home_();

  // Auxiliary node for hosting the home service
  rclcpp::Node::SharedPtr service_node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  void home_service_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response);

  // Controller reset after homing
  std::string controller_name_{"volcaniarm_controller"};
  bool reset_controller_();
};

}  // namespace volcaniarm_hardware
