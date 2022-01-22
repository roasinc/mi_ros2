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

#ifndef MI_ROS__MI_PARSER_HPP_
#define MI_ROS__MI_PARSER_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <bitset>

#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std;

enum ParseState
{
  WAIT_FOR_HEADER1,
  WAIT_FOR_HEADER2,
  WAIT_FOR_INDEX,
  WAIT_FOR_DATA,
  WAIT_FOR_RESERVED,
  WAIT_FOR_CHECKSUM
};

class Parser
{
public:
  Parser(geometry_msgs::msg::Vector3Stamped& rpy, sensor_msgs::msg::Imu& imu, bool tf_ned_to_enu);

  virtual ~Parser() = default;

  /**
   * \brief Parse the message
   * \param msg Message read through serial communication
   */
  void parse(uint8_t msg);

  /**
   * \brief Clear
   */
  void clear();

  /**
   * \brief Convert to IMU sensor data
   * \param data Sensor data read through serial communication
   */
  void convert(const uint8_t* data);

private:
  geometry_msgs::msg::Vector3Stamped& rpy_;
  sensor_msgs::msg::Imu& imu_;

  bool tf_ned_to_enu_;

  uint8_t data_[18];
  int16_t data_cnt_, reserved_cnt_, checksum_;

  ParseState parse_state_;
};

#endif  // MI_ROS__MI_PARSER_HPP_