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