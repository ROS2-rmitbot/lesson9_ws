// These prevent the file from being included more than once during compilation
#ifndef RMITBOT_INTERFACE_HPP
#define RMITBOT_INTERFACE_HPP

// Include necessary headers
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <libserial/SerialPort.h>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>



#include <vector>
#include <string>

namespace rmitbot_firmware
{
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  class RmitbotInterface : public hardware_interface::SystemInterface
  {
  public:
    // Constructor: it actually do nothing, but it is required as part of the hardware interface. Prepare empty boxes
    // Destructor: closes the serial port if it is open. Throw the boxes away
    RmitbotInterface();
    virtual ~RmitbotInterface();

    // on_init: initializes the serial port, buffers, and variables. Label the boxes and prepare them for use
    // on_activate: open the serial port. Start using the boxes
    // on_deactivate: close the serial port. Stop using the boxes
    // These are core nodes for implementing ros2_control lifecycle
    CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
    
    // StateInterface: position and velocity of each wheel, IMU orientation, angular velocity, and linear acceleration
    // CommandInterface: velocity command of each wheel
    // Read function: Read the data from the Arduino and update the state variables
    // Write function: Send the velocity commands to the Arduino
    // These are core nodes for implementing ros2_control hardware interface
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

  private:
    LibSerial::SerialPort arduino_;
    std::string port_;
    std::vector<double> velocity_commands_;
    std::vector<double> position_states_;
    std::vector<double> velocity_states_;
    std::vector<double> orientation_;
    std::vector<double> ang_vel_;
    std::vector<double> lin_acc_;

    rclcpp::Time last_run_;
  };
} // namespace rmitbot_firmware

#endif // RMITBOT_INTERFACE_HPP