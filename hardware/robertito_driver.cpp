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

  RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"),
      "RobertitoDriver::on_init() - BEGIN");

  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobertitoDriver"),
      "RobertitoDriver::on_init() - Failed to initialize");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 8)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("RobertitoDriver"),
      "RobertitoDriver::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 8.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("steering") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("RobertitoDriver"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

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
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
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
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("RobertitoDriver"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RobertitoDriver"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RobertitoDriver"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RobertitoDriver"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RobertitoDriver"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("RobertitoDriver"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  hw_interfaces_["traction"].emplace_back("front_left_wheel_joint");
  hw_interfaces_["traction"].emplace_back("front_right_wheel_joint");
  hw_interfaces_["traction"].emplace_back("rear_left_wheel_joint");
  hw_interfaces_["traction"].emplace_back("rear_right_wheel_joint");
  
  hw_interfaces_["steering"].emplace_back("front_left_steering_joint");
  hw_interfaces_["steering"].emplace_back("front_right_steering_joint");
  hw_interfaces_["steering"].emplace_back("rear_left_steering_joint");
  hw_interfaces_["steering"].emplace_back("rear_right_steering_joint");


  RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"),
      "RobertitoDriver::on_init() - SUCCESS");

  return hardware_interface::CallbackReturn::SUCCESS; 
}

std::vector<hardware_interface::StateInterface>
RobertitoDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joints : hw_interfaces_)
  {
    for(auto & joint : joints.second) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.joint_name, hardware_interface::HW_IF_POSITION, &joint.state.position));
    }

    if (joints.first == "traction")
    {
      for(auto & joint : joints.second) {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
          joint.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.state.velocity));
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RobertitoDriver"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RobertitoDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joints : hw_interfaces_)
  {
    if (joints.first == "steering")
    {
      for(auto & joint : joints.second) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.joint_name, hardware_interface::HW_IF_POSITION,
          &joint.command.position));
      }
    }
    else if (joints.first == "traction")
    {
      for(auto & joint : joints.second) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
          joint.joint_name, hardware_interface::HW_IF_VELOCITY,
          &joint.command.velocity));
      }
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("RobertitoDriver"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("RobertitoDriver"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RobertitoDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Activating ...please wait...");

  for (auto & joints : hw_interfaces_)
  {
      for(auto & joint : joints.second) {
    	joint.state.position = 0.0;

    	if (joints.first == "traction")
    	{
      	  joint.state.velocity = 0.0;
          joint.command.velocity = 0.0;
        }
        else if (joints.first == "steering")
        {
          joint.command.position = 0.0;
        }
      }
  }
  
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobertitoDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Deactivating ...please wait...");

  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RobertitoDriver"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobertitoDriver::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
    
  for (auto & joints : hw_interfaces_) 
  {
    if (joints.first == "steering") {
    	for (auto &joint : joints.second) {
		joint.state.position = joint.command.position;
		RCLCPP_INFO(
    			rclcpp::get_logger("RobertitoDriver"), "Got position state: %.2f for joint '%s'.",
    			joint.command.position, joint.joint_name.c_str());

	}
    }
    if (joints.first == "traction") {
    	for (auto &joint : joints.second) {
		joint.state.velocity= joint.command.velocity;
		joint.state.position = joint.command.velocity * period.seconds();
		RCLCPP_INFO(
    			rclcpp::get_logger("RobertitoDriver"), "Got velocity state: %.2f for joint '%s'.",
    			joint.command.velocity, joint.joint_name.c_str());
  	}
    }
  }

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobertitoDriver::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
 
  for (auto & joints : hw_interfaces_) 
  {
    if (joints.first == "steering") {
    	for (auto &joint : joints.second) {
		RCLCPP_INFO(
    			rclcpp::get_logger("RobertitoDriver"), "Got position command: %.2f for joint '%s'.",
    			joint.command.position, joint.joint_name.c_str());

	}
    }
    if (joints.first == "traction") {
    	for (auto &joint : joints.second) {
		RCLCPP_INFO(
    			rclcpp::get_logger("RobertitoDriver"), "Got velocity command: %.2f for joint '%s'.",
    			joint.command.velocity, joint.joint_name.c_str());
  	}
    }
  }

  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

}  // namespace robertito

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  robertito::RobertitoDriver, hardware_interface::SystemInterface)
