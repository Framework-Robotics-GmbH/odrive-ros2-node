/* BSD 3-Clause License

Copyright (c) 2021, Framework Robotics GmbH
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <sys/file.h>
#include <stdexcept>
#include <regex>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <stdlib.h>
#include "rclcpp/rclcpp.hpp"

class ODrive
{
public:
    ODrive(const std::string port, const rclcpp::Node *node);
    ~ODrive();

    // Commands
    int setVelocity(const int motor, const float velocity);
    int stop();

    // Getters
    std::pair<float, float> getPosition_Velocity(const int motor_number);
    float getBusVoltage();
    float getTemperature(const int motor);
    float getTorque(const int motor);

    // General helper functions
    int checksum(const std::string cmd);

private:
    int open_port(const std::string port);
    int send(const std::string cmd, const std::string *funct_name);
    std::string recieve(const std::string *funct_name);
    bool check_single_output(const std::string s, const std::string *funct_name);
    bool check_double_output(const std::string s, const std::string *funct_name);
    int fd;
    const rclcpp::Node *node;
};
