
#include "walker.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "walk_roomba");
  walkerRoomba run;
  run.runRobot();

  return 0;
}