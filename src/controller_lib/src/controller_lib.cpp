#include "controller_lib/controller_lib.h"

#include <iostream>
#include <string>
#include <vector>

Controller::Controller(std::string port, int baud_rate)
    : io_context_(), controller_(io_context_)
{
    drivers::serial_driver::SerialPortConfig config(
        baud_rate,
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE);

    controller_.init_port(port, config);

    controller_.port()->open();

    if (!controller_.port() || !controller_.port()->is_open())
    {
        std::cerr << "Couldn't open port" << std::endl;
    }
}

Controller::~Controller()
{
    if (controller_.port()->is_open())
    {
        controller_.port()->close();
    }
}

bool Controller::connectedToPort()
{
    if (!controller_.port()->is_open())
    {
        return false;
    }
    return true;
}

int Controller::sendAngles(float theta_R, float theta_W, float theta_G1, float theta_G2)
{
    std::vector<float> command(4);
    command[0] = theta_R;
    command[1] = theta_W;
    command[2] = theta_G1;
    command[3] = theta_G2;

    // Serialize floats to bytes
    std::vector<unsigned char> byte_command(command.size() * sizeof(float));
    std::memcpy(byte_command.data(), command.data(), byte_command.size());

    controller_.port()->send(byte_command);

    std::cout << "Command sent" << std::endl;

    // controller_.flushOutput();   -> NO flushOutput function in new lib

    return 1;
}

std::vector<float> Controller::readPositions()
{
    std::vector<float> positions;
    std::string data;
    constexpr size_t buffer_size = 64;
    std::vector<uint8_t> buffer(buffer_size);

    if (!controller_.port() || !controller_.port()->is_open())
    {
        std::cerr << "Serial port not open." << std::endl;
        return positions;
    }

    try
    {
        while (true)
        {
            size_t bytes_read = controller_.port()->receive(buffer);

            if (bytes_read == 0)
            {
                std::cerr << "No data to read." << std::endl;
                break;
            }

            data.append(reinterpret_cast<char *>(buffer.data()), bytes_read);

            // Find and remove '\n'
            size_t newline_pos = data.find('\n');
            if (newline_pos != std::string::npos)
            {
                std::string line = data.substr(0, newline_pos);
                std::cout << "Positions received: " << line << std::endl;
                positions = parsePositions(line);
                break;
            }
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "receive error: " << e.what() << std::endl;
    }

    return positions;
}

std::vector<float> Controller::parsePositions(std::string angles_str)
{
    std::vector<float> current_positions;
    std::string angle;
    std::stringstream ss(angles_str);

    for (int i = 0; i < num_motors; i++)
    {
        if (getline(ss, angle, ','))
        {
            current_positions.push_back(std::stod(angle));
        }
    }

    return current_positions;
}

void Controller::printPositions(std::vector<float> &positions)
{
    for (size_t i = 0; i < positions.size(); i++)
    {
        std::cout << "Motor " << (i + 1) << " position = " << positions[i] << std::endl;
    }
}
