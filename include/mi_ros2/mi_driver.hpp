/*
Software License Agreement (BSD License)

Authors : Brighten Lee <shlee@roas.co.kr>

Copyright (c) 2021, ROAS Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef MI_ROS__MI_DRIVER_HPP_
#define MI_ROS__MI_DRIVER_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "serial/serial.h"
#include "realtime_tools/realtime_publisher.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "mi_ros2/parser.hpp"

class MiDriver : public rclcpp::Node
{
public:
  MiDriver(const std::string& node);

  virtual ~MiDriver();

  /**
   * \brief Initialize
   */
  bool init();

  /**
   * \brief Read message
   */
  void read();

  /**
   * \brief Reset IMU
   * \param req Request
   * \param resp Response
   */
  void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
             std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  /**
   * \brief Publish IMU sensor data
   */
  void publishData();

  /// Class for serial communication
  std::shared_ptr<serial::Serial> serial_;

  /// Class for parsing
  std::shared_ptr<Parser> parser_;

private:
  /// ROS2 parameters
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;

  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> rp_rpy_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> rp_imu_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_;

  geometry_msgs::msg::Vector3Stamped rpy_;
  sensor_msgs::msg::Imu imu_;

  // Timer for publisher
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// Serial parameters
  std::string port_;
  int32_t baud_;

  /// Publishing rate
  int rate_;

  /// Frame ID
  std::string frame_id_;

  bool tf_ned_to_enu_;
};

#endif  // MI_ROS__MI_DRIVER_HPP_