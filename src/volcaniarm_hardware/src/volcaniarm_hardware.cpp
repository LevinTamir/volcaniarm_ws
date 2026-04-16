#include "volcaniarm_hardware/volcaniarm_hardware.hpp"

#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

#include "pluginlib/class_list_macros.hpp"

namespace volcaniarm_hardware
{

namespace
{
speed_t baud_to_speed_t(int baud)
{
  switch (baud)
  {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    default:      return B115200;
  }
}
}  // namespace

hardware_interface::CallbackReturn VolcaniArmHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Parameters from ros2_control <hardware> tag
  port_ = info_.hardware_parameters.at("serial_port");   // e.g. /dev/ttyUSB0
  baud_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  steps_per_rev_ = std::stod(info_.hardware_parameters.at("steps_per_rev"));

  // Home offset: maps physical zero to URDF zero (default 0.0 = no offset)
  if (info_.hardware_parameters.count("right_elbow_home_offset")) {
    right_elbow_home_offset_ = std::stod(info_.hardware_parameters.at("right_elbow_home_offset"));
  }
  if (info_.hardware_parameters.count("left_elbow_home_offset")) {
    left_elbow_home_offset_ = std::stod(info_.hardware_parameters.at("left_elbow_home_offset"));
  }

  std::cout << "[VolcaniArmHardware] Initialized with steps_per_rev = " << steps_per_rev_ << std::endl;

  hw_position_right_elbow_ = 0.0;
  hw_velocity_right_elbow_ = 0.0;
  hw_position_command_right_elbow_ = 0.0;

  hw_position_left_elbow_ = 0.0;
  hw_velocity_left_elbow_ = 0.0;
  hw_position_command_left_elbow_ = 0.0;

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
VolcaniArmHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Right elbow joint
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "volcaniarm_right_elbow_joint", hardware_interface::HW_IF_POSITION, &hw_position_right_elbow_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "volcaniarm_right_elbow_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocity_right_elbow_));

  // Left elbow joint
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "volcaniarm_left_elbow_joint", hardware_interface::HW_IF_POSITION, &hw_position_left_elbow_));
  state_interfaces.emplace_back(
    hardware_interface::StateInterface(
      "volcaniarm_left_elbow_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocity_left_elbow_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
VolcaniArmHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      "volcaniarm_right_elbow_joint", hardware_interface::HW_IF_POSITION, &hw_position_command_right_elbow_));

  command_interfaces.emplace_back(
    hardware_interface::CommandInterface(
      "volcaniarm_left_elbow_joint", hardware_interface::HW_IF_POSITION, &hw_position_command_left_elbow_));

  return command_interfaces;
}

hardware_interface::CallbackReturn
VolcaniArmHardware::on_configure(const rclcpp_lifecycle::State &)
{
  // Open serial port
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0)
  {
    std::cerr << "[VolcaniArmHardware] Failed to open " << port_
              << ": " << std::strerror(errno) << std::endl;
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!configure_port_())
  {
    std::cerr << "[VolcaniArmHardware] Failed to configure " << port_ << std::endl;
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  ::usleep(200000);  // small delay for Arduino reset

  if (!home_()) {
    std::cerr << "[VolcaniArmHardware] Homing failed" << std::endl;
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Create auxiliary node with home service
  service_node_ = rclcpp::Node::make_shared("volcaniarm_hardware_services");
  home_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
    "volcaniarm_hardware/home",
    std::bind(&VolcaniArmHardware::home_service_callback_, this,
              std::placeholders::_1, std::placeholders::_2));

  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor_->add_node(service_node_);
  executor_thread_ = std::thread([this]() { executor_->spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
VolcaniArmHardware::on_activate(const rclcpp_lifecycle::State &)
{
  hw_position_right_elbow_ = right_elbow_home_offset_;
  hw_velocity_right_elbow_ = 0.0;
  hw_position_command_right_elbow_ = right_elbow_home_offset_;

  hw_position_left_elbow_ = left_elbow_home_offset_;
  hw_velocity_left_elbow_ = 0.0;
  hw_position_command_left_elbow_ = left_elbow_home_offset_;

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
VolcaniArmHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (executor_) {
    executor_->cancel();
  }
  if (executor_thread_.joinable()) {
    executor_thread_.join();
  }
  home_service_.reset();
  executor_.reset();
  service_node_.reset();

  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type
VolcaniArmHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  read_serial_();

  double dt = period.seconds();
  if (dt > 0.0) {
    hw_velocity_right_elbow_ = (hw_position_right_elbow_ - prev_position_right_elbow_) / dt;
    hw_velocity_left_elbow_ = (hw_position_left_elbow_ - prev_position_left_elbow_) / dt;
  }
  prev_position_right_elbow_ = hw_position_right_elbow_;
  prev_position_left_elbow_ = hw_position_left_elbow_;

  return hardware_interface::return_type::OK;
}

void VolcaniArmHardware::read_serial_()
{
  if (fd_ < 0) return;

  // Read available bytes into buffer
  char tmp[128];
  ssize_t n = ::read(fd_, tmp, sizeof(tmp));
  if (n <= 0) return;

  for (ssize_t i = 0; i < n; i++) {
    char c = tmp[i];
    if (c == '\r') continue;

    if (c == '\n') {
      serial_buf_[serial_buf_pos_] = '\0';

      // Parse position report: "S <steps1> <steps2>"
      long steps1 = 0, steps2 = 0;
      if (std::sscanf(serial_buf_, "S %ld %ld", &steps1, &steps2) == 2) {
        double rad_per_step = (2.0 * M_PI) / steps_per_rev_;
        hw_position_right_elbow_ = steps1 * rad_per_step + right_elbow_home_offset_;
        hw_position_left_elbow_ = steps2 * rad_per_step + left_elbow_home_offset_;
      }

      serial_buf_pos_ = 0;
    } else {
      if (serial_buf_pos_ < SERIAL_BUF_SIZE - 1) {
        serial_buf_[serial_buf_pos_++] = c;
      } else {
        serial_buf_pos_ = 0;
      }
    }
  }
}

hardware_interface::return_type
VolcaniArmHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!send_position_command_rad_(hw_position_command_right_elbow_, hw_position_command_left_elbow_))
  {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

bool VolcaniArmHardware::configure_port_()
{
  struct termios tio;
  if (::tcgetattr(fd_, &tio) != 0)
  {
    std::cerr << "[VolcaniArmHardware] tcgetattr failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  ::cfmakeraw(&tio);
  speed_t speed = baud_to_speed_t(baud_);
  if (::cfsetispeed(&tio, speed) != 0 || ::cfsetospeed(&tio, speed) != 0)
  {
    std::cerr << "[VolcaniArmHardware] cfsetispeed/cfsetospeed failed: "
              << std::strerror(errno) << std::endl;
    return false;
  }

  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cflag &= ~PARENB;
  tio.c_cflag &= ~CSTOPB;
  tio.c_cflag &= ~CSIZE;
  tio.c_cflag |= CS8;
  tio.c_cflag &= ~CRTSCTS;

  tio.c_lflag = 0;
  tio.c_iflag = 0;
  tio.c_oflag = 0;

  tio.c_cc[VMIN]  = 0;
  tio.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
  {
    std::cerr << "[VolcaniArmHardware] tcsetattr failed: " << std::strerror(errno) << std::endl;
    return false;
  }

  return true;
}

bool VolcaniArmHardware::send_position_command_rad_(double position_rad_right, double position_rad_left)
{
  if (fd_ < 0)
  {
    std::cerr << "[VolcaniArmHardware] Serial not open" << std::endl;
    return false;
  }

  // rad -> steps for both motors
  double steps_double_right = position_rad_right * (steps_per_rev_ / (2.0 * M_PI));
  long steps_right = std::lround(steps_double_right);

  double steps_double_left = position_rad_left * (steps_per_rev_ / (2.0 * M_PI));
  long steps_left = std::lround(steps_double_left);

  // Send command: "P1 steps_right P2 steps_left\n"
  // P1 = right elbow, P2 = left elbow
  std::string line = "P1 " + std::to_string(steps_right) + " P2 " + std::to_string(steps_left) + "\n";

  const char * data = line.c_str();
  ssize_t len = static_cast<ssize_t>(line.size());
  ssize_t written = 0;

  while (written < len)
  {
    ssize_t n = ::write(fd_, data + written, len - written);
    if (n < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }
      std::cerr << "[VolcaniArmHardware] write failed: " << std::strerror(errno) << std::endl;
      return false;
    }
    written += n;
  }

  return true;
}

void VolcaniArmHardware::home_service_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
  std_srvs::srv::Trigger::Response::SharedPtr response)
{
  if (home_()) {
    response->success = true;
    response->message = "Homing complete, positions reset to 0,0";
  } else {
    response->success = false;
    response->message = "Homing failed: serial write error";
  }
}

bool VolcaniArmHardware::home_()
{
  // TODO: implement full homing with limit switches.
  // For now, send 'H' to the ESP which resets its position to 0,0.
  const char cmd[] = "H\n";
  ssize_t n = ::write(fd_, cmd, sizeof(cmd) - 1);
  if (n < 0) {
    std::cerr << "[VolcaniArmHardware] home command write failed: "
              << std::strerror(errno) << std::endl;
    return false;
  }

  hw_position_right_elbow_ = 0.0;
  hw_position_command_right_elbow_ = 0.0;
  hw_position_left_elbow_ = 0.0;
  hw_position_command_left_elbow_ = 0.0;

  std::cout << "[VolcaniArmHardware] Home: positions set to 0,0" << std::endl;
  return true;
}

}  // namespace volcaniarm_hardware

PLUGINLIB_EXPORT_CLASS(
  volcaniarm_hardware::VolcaniArmHardware,
  hardware_interface::SystemInterface)
