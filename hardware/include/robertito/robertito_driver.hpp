#ifndef ROBERTITO__ROBERTITO_DRIVER_HPP_
#define ROBERTITO__ROBERTITO_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"

namespace robertito
{
struct JointValue
{
  double position{0.0};
  double velocity{0.0};
  double effort{0.0};
};

struct Joint
{
  explicit Joint(const std::string & name, const std::string & driver_name) : joint_name(name), driver_name(driver_name)
  {
    state = JointValue();
    command = JointValue();
  }

  Joint() = default;

  std::string joint_name;
  std::string driver_name;
  JointValue state;
  JointValue command;
};

class RobertitoDriverNode: public rclcpp::Node  //the node definition for the publisher to talk to micro-ROS agent
{
  public:
    RobertitoDriverNode();
    void publish(const std::string& driver_name, geometry_msgs::msg::TwistStamped message);

  private:
    std::map<std::string, rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr> driver_pubs_; 

};

class RobertitoDriver: public hardware_interface::SystemInterface, public rclcpp::Node
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobertitoDriver)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

 std::map<std::string, std::vector<Joint>> hw_interfaces_;
 std::shared_ptr<RobertitoDriverNode> node_;
};

}  // namespace robertito

#endif  // ROBERTITO__ROBERTITO_DRIVER_HPP_
