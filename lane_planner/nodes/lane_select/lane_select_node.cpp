/*
判断当前车道，同时规划从当前车道切换至其他车道的轨迹，
接着根据话题"state"中的驾驶状态(是否需要换道)发布当前车道数/
换道轨迹数据至话题"base_waypoints"供其他节点继续规划。
 */

// ROS includes
#include <ros/ros.h>

#include "lane_select_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_select");
  lane_planner::LaneSelectNode lsn;
  lsn.run();

  return 0;
}
