#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

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

  bool configure_port_();
  bool send_position_command_rad_(double position_rad_right, double position_rad_left);
};

}  // namespace volcaniarm_hardware
