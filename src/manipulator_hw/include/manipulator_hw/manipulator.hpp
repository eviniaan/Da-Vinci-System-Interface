#ifndef MANIPULATOR_HPP
#define MANIPULATOR_HPP

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/hardware_info.hpp> // struct containing data from URDF's <ros2_control> tag

#include "controller_lib/controller_lib.h"

#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace manipulator_hw
{
    class Manipulator : public hardware_interface::SystemInterface
    {
    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Manipulator)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        // State and command export functions
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        // Activate and deactivate functions
        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        // Read and write functions
        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    private:
        // State and command interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces_;
        std::vector<hardware_interface::CommandInterface> command_interfaces_;
        std::vector<double> hw_commands_; // might have to change variable name to be more specific
        std::vector<double> hw_positions_;


        // From the class of the controller library
        std::unique_ptr<Controller> openrb_controller_; // had to do it like this bcs my class contains SerialDriver and IoContext which are non-copyable members
        //Controller openrb_controller_;
    };
}

#endif
