# mi_ros2
**This repository will not be maintained further.**

## Overview
ROS2 driver for XG6000 IMU sensor

ROS2 Interface
-----------

| Topic Name   | Type                             | Description             |
|--------------|----------------------------------|-------------------------|
| ``imu/data`` | ``sensor_msgs/Imu``              | IMU values              |
| ``imu/rpy``  | ``geometry_msgs/Vector3Stamped`` | Roll. Pitch, Yaw values |

| Service Name  | Type                 | Description      |
|---------------|----------------------|------------------|
| ``imu/reset`` | ``std_srvs/Trigger`` | Reset the sensor |

Installation
------------

```
cd ~/catkin_ws/src/
git clone https://github.com/wjwwood/serial.git -b ros2
git clone https://github.com/roasinc/mi_ros2.git

cd ~/catkn_ws/
rosdep install --from-paths src --ignore-src -y
catkin_make
```
