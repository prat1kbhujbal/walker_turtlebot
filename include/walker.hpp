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
 * @file walker.hpp
 * @author Pratik Bhujbal
 * @brief  class for obstacle avoidance for turtlebot3 
 * @version 1.0
 * @date 11/23/2021
 * @copyright  Copyright (c) 2021
 * 
 */
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class walkerRoomba {
 public:
  /**
   * @brief      Constructor of the class
   */
  walkerRoomba();
  /**
   * @brief      Destructor of the class
   */
  ~walkerRoomba();
  /**
   * @brief      Callback for laser data
   *
   * @param message to check collision
   *
   * @return     void: Return nothing
   */
  auto callbackObstacle(const sensor_msgs::LaserScan::ConstPtr& msg) -> void;
 /**
   * @brief Mvve inside world until collision is detected
   */
  auto runRobot() -> void;



 private:
   int _left_flag;             // left flag to check the collision at right and to move robot left
   int _right_flag;           // / right flag to check the collision at left and to move robot right
   ros::NodeHandle _n;        // node handle for the class
    /**
   * @brief publisher object for the /cmd_vel topic
   * 
   */
   ros::Publisher _pub;       
   /**
   * @brief subsriber to the /scan topic
   * 
   */
   ros::Subscriber _sub;     
   
   geometry_msgs::Twist _vel; // velocity message
};

#endif  // INCLUDE_WALKER_HPP_