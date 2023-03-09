/**
 * 从本地文件加载采集的轨迹点
*/
// ROS Includes
#include <ros/ros.h>

#include "waypoint_loader_core.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_loader");
  waypoint_maker::WaypointLoaderNode wln;
  wln.run();

  return 0;
}
