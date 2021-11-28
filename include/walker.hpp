
#ifndef INCLUDE_WALKER_HPP_
#define INCLUDE_WALKER_HPP_

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class walkerRoomba {
 public:
 
  walkerRoomba();
  
  ~walkerRoomba();

  auto callbackObstacle(const sensor_msgs::LaserScan::ConstPtr& msg) -> void;

  auto runRobot() -> void;



 private:
   int _left_flag;             
   int _right_flag;         
   ros::NodeHandle _n;        
   ros::Publisher _pub;       
   ros::Subscriber _sub;     
   
   geometry_msgs::Twist _vel;
};

#endif  // INCLUDE_WALKER_HPP_