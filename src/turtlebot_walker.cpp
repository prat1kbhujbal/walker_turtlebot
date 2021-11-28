/**
 * MIT License
 *
 * Copyright (c) 2021 Pratik Bhujbal
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to 
 * deal in the Software without restriction, including without limitation the  
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or 
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING 
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE. 
 * 
 * @file turtlebot_walker.cpp
 * @author Pratik Bhujbal
 * @brief  Class implementation
 * @version 1.0
 * @date 11/23/2021
 * @copyright  Copyright (c) 2021
 * 
 */
#include "walker.hpp"

walkerRoomba::walkerRoomba() {
  // Publisher for velocity commands
  _pub = _n.advertise < geometry_msgs::Twist > ("/cmd_vel", 1000);
  // Subscribe to laser scan data
  _sub = _n.subscribe < sensor_msgs::LaserScan
      > ("/scan", 1000, &walkerRoomba::callbackObstacle, this);

// Initialize both the flags to zero
  _left_flag = 0;
  _right_flag = 0;
}

walkerRoomba::~walkerRoomba() {
  // publish zero the velocity while destructing the object
  _vel.linear.x = 0;
  _vel.angular.z = 0;
  _pub.publish(_vel);
}

auto walkerRoomba::callbackObstacle(
    const sensor_msgs::LaserScan::ConstPtr &msg) -> void {
  _left_flag = 0;
  _right_flag = 0;
  for (int i = 0; i < (msg->ranges.size()); i++) {
    // check for obstacles
    if (msg->ranges[0] < 0.7 || msg->ranges[25] < 0.5) {
      _right_flag = 1;
      _left_flag = 0;
      ROS_WARN_STREAM("Obstacle at Left");
    } else if (msg->ranges[0] < 0.7 || msg->ranges[335] < 0.5) {
      _left_flag = 1;
      _right_flag = 0;
      ROS_WARN_STREAM("Obstacle at Right");
    }
  }
}

auto walkerRoomba::runRobot() -> void {
  // Initialize the rate at which the messages will be published
  ros::Rate rate(10.0);
  while (ros::ok()) {
    if (_right_flag == 1) {
      _vel.linear.x = 0.0;
      _vel.angular.z = -0.5;
      ROS_INFO_STREAM("Turning Right");
    } else if (_left_flag == 1) {
      _vel.linear.x = 0.0;
      _vel.angular.z = 0.5;
      ROS_INFO_STREAM("Turning Left");
    } else {
      _vel.linear.x = 0.2;
      _vel.angular.z = 0.0;
      ROS_INFO_STREAM("Moving Forward");
    }
    // Publish the velocity
    _pub.publish(_vel);
    ros::spinOnce();
    rate.sleep();
  }
}
