#ifndef DAVINCI_MANIPULATOR_H
#define DAVINCI_MANIPULATOR_H

#include "serial_driver/serial_driver.hpp"

#include "io_context/io_context.hpp"

#include <string>
#include <vector>

class Controller
{
    public:
        Controller(std::string port, int baud_rate);
        ~Controller();

        bool connectedToPort();
        int sendAngles(float theta_R, float theta_W, float theta_G1, float theta_G2);
        std::vector<float> readPositions();
        std::vector<float> parsePositions(std::string angles_str);
        void printPositions(std::vector<float>& positions);


    private:
        drivers::common::IoContext io_context_;
        drivers::serial_driver::SerialDriver controller_;
        int const num_motors = 4;

};

#endif
