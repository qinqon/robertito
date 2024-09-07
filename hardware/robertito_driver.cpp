#include "robertito/robertito_driver.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robertito
{
hardware_interface::CallbackReturn RobertitoDriver::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobertitoDriver"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobertitoDriver"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobertitoDriver"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobertitoDriver"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // RobertitoDriver has exactly two GPIO components
  if (info_.gpios.size() != 2)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RobertitoDriver"),
      "RobertitoDriver has '%ld' GPIO components, '%d' expected.", info_.gpios.size(),
      2);
    return hardware_interface::CallbackReturn::ERROR;
  }
  // with exactly 1 command interface
  for (int i = 0; i < 2; i++)
  {
    if (info_.gpios[i].command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RobertitoDriver"),
        "GPIO component %s has '%ld' command interfaces, '%d' expected.",
        info_.gpios[i].name.c_str(), info_.gpios[i].command_interfaces.size(), 1);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  // and 3/1 state interfaces, respectively
  if (info_.gpios[0].state_interfaces.size() != 3)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RobertitoDriver"),
      "GPIO component %s has '%ld' state interfaces, '%d' expected.", info_.gpios[0].name.c_str(),
      info_.gpios[0].state_interfaces.size(), 3);
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.gpios[1].state_interfaces.size() != 1)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("RobertitoDriver"),
      "GPIO component %s has '%ld' state interfaces, '%d' expected.", info_.gpios[0].name.c_str(),
      info_.gpios[0].state_interfaces.size(), 1);
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobertitoDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Configuring ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // reset values always when configuring hardware
  std::fill(hw_states_.begin(), hw_states_.end(), 0);
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0);
  std::fill(hw_gpio_in_.begin(), hw_gpio_in_.end(), 0);
  std::fill(hw_gpio_out_.begin(), hw_gpio_out_.end(), 0);

  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RobertitoDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "State interfaces:");
  hw_gpio_in_.resize(4);
  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto state_if : info_.gpios.at(i).state_interfaces)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.gpios.at(i).name, state_if.name, &hw_gpio_in_[ct++]));
      RCLCPP_INFO(
        rclcpp::get_logger("RobertitoDriver"), "Added %s/%s",
        info_.gpios.at(i).name.c_str(), state_if.name.c_str());
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobertitoDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Command interfaces:");
  hw_gpio_out_.resize(2);
  size_t ct = 0;
  for (size_t i = 0; i < info_.gpios.size(); i++)
  {
    for (auto command_if : info_.gpios.at(i).command_interfaces)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.gpios.at(i).name, command_if.name, &hw_gpio_out_[ct++]));
      RCLCPP_INFO(
        rclcpp::get_logger("RobertitoDriver"), "Added %s/%s",
        info_.gpios.at(i).name.c_str(), command_if.name.c_str());
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobertitoDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Activating ...please wait...");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  // command and state should be equal when starting
  for (uint i = 0; i < hw_states_.size(); i++)
  {
    hw_commands_[i] = hw_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobertitoDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Successfully deactivated!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobertitoDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]);
  }

  // mirror GPIOs back
  hw_gpio_in_[0] = hw_gpio_out_[0];
  hw_gpio_in_[3] = hw_gpio_out_[1];
  // random inputs
  unsigned int seed = time(NULL) + 1;
  hw_gpio_in_[1] = static_cast<float>(rand_r(&seed));
  seed = time(NULL) + 2;
  hw_gpio_in_[2] = static_cast<float>(rand_r(&seed));

  for (uint i = 0; i < hw_gpio_in_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"), "Read %.1f from GP input %d!",
      hw_gpio_in_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "GPIOs successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobertitoDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Writing...");

  for (uint i = 0; i < hw_gpio_out_.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"), "Got command %.1f for GP output %d!",
      hw_gpio_out_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "GPIOs successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace robertito

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robertito::RobertitoDriver, hardware_interface::SystemInterface)
