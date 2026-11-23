// Open-loop ros2_control hardware interface for the Arduino motor controller.

#include "diffdrive_arduino/diffbot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_arduino
{



// init all params and stored in HW info
hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_init(   
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // PULL OUT params in HW-INFO : 
  // stof/stoi: string to int/float convert
  cfg_.left_wheel_name    = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name   = info_.hardware_parameters["right_wheel_name"];
  cfg_.device             = info_.hardware_parameters["device"];
  cfg_.baud_rate          = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms         = std::stoi(info_.hardware_parameters["timeout_ms"]);
  const auto startup_delay = info_.hardware_parameters.find("startup_delay_ms");
  if (startup_delay != info_.hardware_parameters.end())
  {
    cfg_.startup_delay_ms = std::stoi(startup_delay->second);
  }
  cfg_.max_motor_command = std::stoi(info_.hardware_parameters["max_motor_command"]);
  cfg_.max_wheel_angular_velocity =
    std::stod(info_.hardware_parameters["max_wheel_angular_velocity"]);

  if (cfg_.max_motor_command <= 0 || cfg_.max_motor_command > 255)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "max_motor_command must be in the range 1 to 255.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.max_wheel_angular_velocity <= 0.0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "max_wheel_angular_velocity must be greater than zero.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (cfg_.startup_delay_ms < 0)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "startup_delay_ms must not be negative.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  wheel_l_.setup(cfg_.left_wheel_name);
  wheel_r_.setup(cfg_.right_wheel_name);

  
  // Checking INFO-matching
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}







// Linking wheel params into exposed HW interfaces
std::vector<hardware_interface::StateInterface> DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  return command_interfaces;
}





hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Configuring ...please wait...");
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  try
  {
    comms_.connect(
      cfg_.device, cfg_.baud_rate, cfg_.timeout_ms, cfg_.startup_delay_ms);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Failed to connect to Arduino: %s", exception.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}




hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Cleaning up ...please wait...");
  const bool stopped = !comms_.connected() || stop_motors();
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  if (!stopped)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Arduino did not acknowledge the stop command during cleanup.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}






hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Activating ...please wait...");
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  constexpr int max_stop_attempts = 3;
  bool stopped = false;
  for (int attempt = 1; attempt <= max_stop_attempts; ++attempt)
  {
    if (stop_motors())
    {
      stopped = true;
      break;
    }

    RCLCPP_WARN(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Initial stop command was not acknowledged (attempt %d/%d).",
      attempt, max_stop_attempts);
    if (attempt < max_stop_attempts)
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }
  if (!stopped)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Arduino did not acknowledge the initial stop command.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}





hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Deactivating ...please wait...");
  if (!stop_motors())
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Arduino did not acknowledge the stop command during deactivation.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const bool stopped = !comms_.connected() || stop_motors();
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  return stopped ? hardware_interface::CallbackReturn::SUCCESS
                 : hardware_interface::CallbackReturn::ERROR;
}


hardware_interface::CallbackReturn DiffDriveArduinoHardware::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  const bool stopped = !comms_.connected() || stop_motors();
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  return stopped ? hardware_interface::CallbackReturn::SUCCESS
                 : hardware_interface::CallbackReturn::ERROR;
}


bool DiffDriveArduinoHardware::stop_motors()
{
  wheel_l_.cmd = 0.0;
  wheel_r_.cmd = 0.0;
  wheel_l_.vel = 0.0;
  wheel_r_.vel = 0.0;

  if (!comms_.connected())
  {
    return false;
  }

  try
  {
    return comms_.set_motor_values(0, 0);
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Failed to send stop command: %s", exception.what());
    return false;
  }
}





hardware_interface::return_type DiffDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{

  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  const double delta_seconds = period.seconds();
  wheel_l_.vel = std::clamp(
    wheel_l_.cmd, -cfg_.max_wheel_angular_velocity, cfg_.max_wheel_angular_velocity);
  wheel_r_.vel = std::clamp(
    wheel_r_.cmd, -cfg_.max_wheel_angular_velocity, cfg_.max_wheel_angular_velocity);
  wheel_l_.pos += wheel_l_.vel * delta_seconds;
  wheel_r_.pos += wheel_r_.vel * delta_seconds;

  return hardware_interface::return_type::OK;
}




hardware_interface::return_type DiffDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  const auto to_motor_command = [this](double wheel_command)
  {
    if (!std::isfinite(wheel_command))
    {
      return 0;
    }
    const double normalized = std::clamp(
      wheel_command / cfg_.max_wheel_angular_velocity, -1.0, 1.0);
    return static_cast<int>(std::lround(normalized * cfg_.max_motor_command));
  };

  const int left_motor_command = to_motor_command(wheel_l_.cmd);
  const int right_motor_command = to_motor_command(wheel_r_.cmd);

  try
  {
    if (!comms_.set_motor_values(left_motor_command, right_motor_command))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("DiffDriveArduinoHardware"),
        "Arduino did not acknowledge motor command; stopping hardware interface.");
      stop_motors();
      return hardware_interface::return_type::ERROR;
    }
  }
  catch (const std::exception & exception)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("DiffDriveArduinoHardware"),
      "Failed to send motor command: %s", exception.what());
    stop_motors();
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace diffdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_arduino::DiffDriveArduinoHardware, hardware_interface::SystemInterface)
