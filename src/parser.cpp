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

#include "mi_ros2/parser.hpp"

#define BUFFER_SIZE 26

Parser::Parser(geometry_msgs::msg::Vector3Stamped& rpy, sensor_msgs::msg::Imu& imu, bool tf_ned_to_enu)
  : rpy_(rpy)
  , imu_(imu)
  , tf_ned_to_enu_(tf_ned_to_enu)
  , data_{}
  , data_cnt_(0)
  , reserved_cnt_(0)
  , checksum_(0)
  , parse_state_(WAIT_FOR_HEADER1)
{
}

void Parser::parse(uint8_t msg)
{
  switch (parse_state_)
  {
    case WAIT_FOR_HEADER1: {
      if (msg == 0xa6)
        parse_state_ = WAIT_FOR_HEADER2;
      else
        clear();

      break;
    }
    case WAIT_FOR_HEADER2: {
      if (msg == 0xa6)
        parse_state_ = WAIT_FOR_INDEX;
      else
        clear();

      break;
    }
    case WAIT_FOR_INDEX: {
      parse_state_ = WAIT_FOR_DATA;
      checksum_ += msg;
      break;
    }
    case WAIT_FOR_DATA: {
      data_[data_cnt_] = msg;
      checksum_ += msg;
      data_cnt_++;

      if (data_cnt_ >= 18)
        parse_state_ = WAIT_FOR_RESERVED;

      break;
    }
    case WAIT_FOR_RESERVED: {
      reserved_cnt_++;

      if (reserved_cnt_ >= 4)
        parse_state_ = WAIT_FOR_CHECKSUM;

      break;
    }
    case WAIT_FOR_CHECKSUM: {
      // To remove the overflow byte
      bitset<8> checksum_byte;
      for (int8_t i = 0; i < 8; i++)
        checksum_byte[i] = bitset<8>(checksum_)[i];

      if (unsigned(msg) == checksum_byte.to_ulong())
        convert(data_);

      clear();
      break;
    }
    default:
      break;
  }
}

void Parser::clear()
{
  for (int8_t i = 0; i < 18; i++)
    data_[i] = 0x00;

  data_cnt_ = 0;
  reserved_cnt_ = 0;
  checksum_ = 0;
  parse_state_ = WAIT_FOR_HEADER1;
}

void Parser::convert(const uint8_t* data)
{
  double euler_r = (static_cast<int16_t>((static_cast<uint8_t>(data[1]) << 8) | static_cast<uint8_t>(data[0]))) /
                   100.0 * M_PI / 180.0;
  double euler_p = (static_cast<int16_t>((static_cast<uint8_t>(data[3]) << 8) | static_cast<uint8_t>(data[2]))) /
                   100.0 * M_PI / 180.0;
  double euler_y = (static_cast<int16_t>((static_cast<uint8_t>(data[5]) << 8) | static_cast<uint8_t>(data[4]))) /
                   100.0 * M_PI / 180.0;

  double gyro_r = (static_cast<int16_t>((static_cast<uint8_t>(data[7]) << 8) | static_cast<uint8_t>(data[6]))) / 100.0 *
                  M_PI / 180.0;
  double gyro_p = (static_cast<int16_t>((static_cast<uint8_t>(data[9]) << 8) | static_cast<uint8_t>(data[8]))) / 100.0 *
                  M_PI / 180.0;
  double gyro_y = (static_cast<int16_t>((static_cast<uint8_t>(data[11]) << 8) | static_cast<uint8_t>(data[10]))) /
                  100.0 * M_PI / 180.0;

  double accel_x =
      (static_cast<int16_t>((static_cast<uint8_t>(data[13]) << 8) | static_cast<uint8_t>(data[12]))) / 1000.0 * 9.80665;
  double accel_y =
      (static_cast<int16_t>((static_cast<uint8_t>(data[15]) << 8) | static_cast<uint8_t>(data[14]))) / 1000.0 * 9.80665;
  double accel_z =
      (static_cast<int16_t>((static_cast<uint8_t>(data[17]) << 8) | static_cast<uint8_t>(data[16]))) / 1000.0 * 9.80665;

  tf2::Quaternion quat;
  quat.setRPY(euler_r, euler_p, euler_y);

  if (tf_ned_to_enu_)
  {
    rpy_.vector.x = euler_p;
    rpy_.vector.y = euler_r;
    rpy_.vector.z = -euler_y;

    imu_.orientation.x = quat[0];
    imu_.orientation.y = -quat[2];
    imu_.orientation.z = quat[3];
    imu_.orientation.w = quat[1];

    imu_.angular_velocity.x = gyro_r;
    imu_.angular_velocity.y = -gyro_p;
    imu_.angular_velocity.z = -gyro_y;

    imu_.linear_acceleration.x = accel_x;
    imu_.linear_acceleration.y = -accel_y;
    imu_.linear_acceleration.z = -accel_z;
  }
  else
  {
    rpy_.vector.x = euler_r;
    rpy_.vector.y = euler_p;
    rpy_.vector.z = euler_y;

    imu_.orientation.x = quat[0];
    imu_.orientation.y = quat[1];
    imu_.orientation.z = quat[2];
    imu_.orientation.w = quat[3];

    imu_.angular_velocity.x = gyro_r;
    imu_.angular_velocity.y = gyro_p;
    imu_.angular_velocity.z = gyro_y;

    imu_.linear_acceleration.x = accel_x;
    imu_.linear_acceleration.y = accel_y;
    imu_.linear_acceleration.z = accel_z;
  }
}