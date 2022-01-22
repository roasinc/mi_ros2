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

#include "mi_ros2/mi_driver.hpp"

#define BUFFER_SIZE 26
#define RESET "$MIB,RESET*87"

MiDriver::MiDriver(const std::string& node)
  : Node(node), port_("/dev/ttyUSB0"), baud_(38400), rate_(50), frame_id_("imu_link"), tf_ned_to_enu_(true)
{
  this->declare_parameter("port");
  this->declare_parameter("rate");
  this->declare_parameter("frame_id");

  this->get_parameter("port", port_);
  this->get_parameter("rate", rate_);
  this->get_parameter("frame_id", frame_id_);

  serial_ = std::make_shared<serial::Serial>();
  parser_ = std::make_shared<Parser>(rpy_, imu_, tf_ned_to_enu_);
}

MiDriver::~MiDriver()
{
  serial_->close();
}

bool MiDriver::init()
{
  if (rate_ > 100 || rate_ <= 0)
  {
    RCLCPP_WARN(this->get_logger(), "MI driver does not support update rate of %f", rate_);
    return false;
  }

  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "imu/reset", std::bind(&MiDriver::reset, this, std::placeholders::_1, std::placeholders::_2));

  pub_rpy_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 1);
  rp_rpy_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(pub_rpy_);
  rp_rpy_->msg_.header.frame_id = frame_id_;

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  rp_imu_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu_);
  rp_imu_->msg_.header.frame_id = frame_id_;
  rp_imu_->msg_.orientation_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };
  rp_imu_->msg_.angular_velocity_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };
  rp_imu_->msg_.linear_acceleration_covariance = { 0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01 };

  // Initial setting for serial communication
  serial_->setPort(port_);
  serial_->setBaudrate(baud_);
  serial::Timeout to = serial::Timeout::simpleTimeout(1000);
  serial_->setTimeout(to);

  try
  {
    serial_->open();
  }
  catch (serial::IOException& e)
  {
    RCLCPP_ERROR_STREAM_ONCE(this->get_logger(), "serial::IOException: " << e.what());
  }

  // Check whether the serial port is open or not
  if (serial_->isOpen())
  {
    RCLCPP_INFO(this->get_logger(), "MI driver connected to %s at %i baud", port_.c_str(), baud_);
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "MI driver failed to connect to %s", port_.c_str());
    return false;
  }

  auto timer_callback = [this]() -> void { this->publishData(); };
  publish_timer_ = this->create_wall_timer(chrono::milliseconds(1000 / rate_), timer_callback);

  return true;
}

void MiDriver::read()
{
  if (serial_->available())
  {
    uint8_t buffer[BUFFER_SIZE];
    serial_->read(buffer, BUFFER_SIZE);
    for (int8_t i = 0; i < BUFFER_SIZE; i++)
      parser_->parse(buffer[i]);
  }
}

void MiDriver::reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                     std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  stringstream msg;
  msg << RESET << "\r";
  serial_->write(msg.str());
  resp->success = true;
}

void MiDriver::publishData()
{
  if (rp_rpy_->trylock())
  {
    rp_rpy_->msg_.header.stamp = this->now();
    rp_rpy_->msg_.vector = rpy_.vector;
    rp_rpy_->unlockAndPublish();
  }

  if (rp_imu_->trylock())
  {
    rp_imu_->msg_.header.stamp = this->now();
    rp_imu_->msg_.orientation = imu_.orientation;
    rp_imu_->msg_.angular_velocity = imu_.angular_velocity;
    rp_imu_->msg_.linear_acceleration = imu_.linear_acceleration;
    rp_imu_->unlockAndPublish();
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = make_shared<MiDriver>("mi_driver_node");

  if (node->init())
  {
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    std::thread t([&executor]() -> void { executor->spin(); });

    while (rclcpp::ok())
      node->read();
  }

  rclcpp::shutdown();
  return 0;
}