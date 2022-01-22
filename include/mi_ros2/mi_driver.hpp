// Copyright (c) 2021, ROAS Inc.
// 
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
  void reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
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

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;

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