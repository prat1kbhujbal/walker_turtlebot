
#include "walker.hpp"

walkerRoomba::walkerRoomba() {
  _pub =
      _n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

  _sub = _n.subscribe<sensor_msgs::LaserScan>(
      "/scan", 1000, &walkerRoomba::callbackObstacle, this);

  _left_flag = 0;
  _right_flag = 0;
}

walkerRoomba::~walkerRoomba() {
  _vel.linear.x = 0;
  _vel.angular.z = 0;
  _pub.publish(_vel);
}

auto walkerRoomba::callbackObstacle(
    const sensor_msgs::LaserScan::ConstPtr& msg) -> void {
     _left_flag = 0;
     _right_flag = 0;
     for (int i = 0; i < (msg->ranges.size()); i++)
     {
    
       if (msg->ranges[0] < 0.7 || msg->ranges[25] < 0.5)
       {
         _right_flag = 1;
         _left_flag = 0;
       }
       else if (msg->ranges[0] < 0.7 || msg->ranges[335] < 0.5)
       {
         _left_flag = 1;
         _right_flag = 0;
       }
     }
}

auto walkerRoomba::runRobot() -> void {
  ros::Rate rate(10.0);
  while (ros::ok()) {
    if (_right_flag==1) {
      _vel.linear.x = 0.0;
      _vel.angular.z = -0.5;
        } else if (_left_flag==1) {
      _vel.linear.x = 0.0;
      _vel.angular.z = 0.5;
        }
    else{
      _vel.linear.x = 0.2;
      _vel.angular.z = 0.0;
    }
    _pub.publish(_vel);
    ros::spinOnce();
    rate.sleep();
  }
}