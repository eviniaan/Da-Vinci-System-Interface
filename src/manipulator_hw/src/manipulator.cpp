#include "manipulator_hw/manipulator.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp" // for hardware_interface::HW_IF_POSITION and hardware_interface::HW_IF_VELOCITY

namespace manipulator_hw
{
    hardware_interface::CallbackReturn Manipulator::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("Init"), "Started init");

        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Init"), "Init failed");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // to retain the URDF data
        info_ = info;

        // Initializing vectors with the correct size and initial value of 0
        hw_commands_.resize(4, 0.0); // 4 = nb of joints
        hw_positions_.resize(4, 0.0);

        for (const hardware_interface::ComponentInfo &joint : info_.joints)
        {
            RCLCPP_INFO(rclcpp::get_logger("Init"), "Joint %s", joint.name.c_str());
            if (joint.command_interfaces.size() != 1 || joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("Init_Components"), "Joint %s needs to have 1 position command interface", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }

            if (joint.state_interfaces.size() != 1 || joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
            {
                RCLCPP_FATAL(rclcpp::get_logger("Init_Components"), "Joint %s needs to have 1 position state interface", joint.name.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        RCLCPP_INFO(rclcpp::get_logger("Init"), "Init successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Manipulator::on_configure(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("Config"), "Started config");

        (void)previous_state; // to help silence warnings of "unused parameter"

        // Read port and baudrate values + Connect to port
        try
        {
            std::string port = info_.hardware_parameters.at("port");
            int baudrate = std::stoi(info_.hardware_parameters.at("baudrate"));
            RCLCPP_INFO(rclcpp::get_logger("Config"), "port and baudrate = %s %d", port.c_str(), baudrate);

            openrb_controller_ = std::make_unique<Controller>(port, baudrate);

            if (!openrb_controller_->connectedToPort())
            {
                RCLCPP_ERROR(rclcpp::get_logger("Config"), "Failed to connect to Arduino");
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        catch (const std::exception &e)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Config"), "Config successful");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    // The following fctn tells ROS what states can be read
    std::vector<hardware_interface::StateInterface> Manipulator::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
        }

        return state_interfaces;
    }

    // This fctn tells ROS what commands can be sent
    std::vector<hardware_interface::CommandInterface> Manipulator::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for (size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
        }

        return command_interfaces;
    }

    hardware_interface::CallbackReturn Manipulator::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        // Since there's no big thing to be done in this function we just check to see if the controller connected
        // to the port and we send the initial angles (all equal to 0) to make sure that the tool is at the right position when starting
        RCLCPP_INFO(rclcpp::get_logger("Activate"), "Activating hardware interface...");

        if (!openrb_controller_ || !openrb_controller_->connectedToPort())
        {
            RCLCPP_ERROR(rclcpp::get_logger("Activate"), "Controller is not connected to the Arduino");
            return hardware_interface::CallbackReturn::ERROR;
        }

        openrb_controller_->sendAngles(hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3]);

        RCLCPP_INFO(rclcpp::get_logger("Activate"), "Hardware interface activated");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Manipulator::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        (void)previous_state;

        // Nothing to deactivate
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Manipulator::read(const rclcpp::Time &time, const rclcpp::Duration &duration)
    {
        // RCLCPP_INFO(rclcpp::get_logger("TEST"), "Hello from read()");

        (void)time;
        (void)duration;

        // The following can be used to measure the jitter - Part 1
        // static rclcpp::Time last_time = time;
        // rclcpp::Duration interval = time - last_time;
        // last_time = time;
        // auto start = std::chrono::steady_clock::now();

        std::vector<float> positions;
        int retries = 0;
        const int max_retries = 5;

        while (positions.size() != 4 && retries < max_retries)
        {
            positions = openrb_controller_->readPositions();
            ++retries;
        }

        if (positions.size() != 4)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Read"), "Failed to get 4 joint positions after %d retries", retries);
            return hardware_interface::return_type::ERROR;
        }

        for (size_t i = 0; i < hw_positions_.size(); ++i)
        {
            hw_positions_[i] = static_cast<double>(positions[i]);
        }

        // The following can be used to measure the jitter - Part 2
        // auto end = std::chrono::steady_clock::now();
        // auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // RCLCPP_INFO(rclcpp::get_logger("Jitter"), "read() execution time: %ld microseconds", duration_us);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Manipulator::write(const rclcpp::Time &time, const rclcpp::Duration &duration)
    {
        // RCLCPP_INFO(rclcpp::get_logger("TEST"), "Hello from write()");

        (void)time;
        (void)duration;

        // The following can be used to measure the jitter - Part 1
        // static rclcpp::Time last_time = time;
        // rclcpp::Duration interval = time - last_time;
        // last_time = time;
        // auto start = std::chrono::steady_clock::now();

        openrb_controller_->sendAngles(
            static_cast<float>(hw_commands_[0]),
            static_cast<float>(hw_commands_[1]),
            static_cast<float>(hw_commands_[2]),
            static_cast<float>(hw_commands_[3]));

        // The following can be used to measure the jitter - Part 2
        // auto end = std::chrono::steady_clock::now();
        // auto duration_us = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // RCLCPP_INFO(rclcpp::get_logger("Jitter"), "write() execution time: %ld microseconds", duration_us);

        return hardware_interface::return_type::OK;
    }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(manipulator_hw::Manipulator, hardware_interface::SystemInterface)