/**
 * @brief global Planner represents the Global planning Module, to generate global plan (reference path) the nodes takes start position, goal position and return sequence of waypoints.
 * @brief Once the node starts (depending on the mapping option) it will load the vector map (road network) and send it to be visualized by RViz. topic name "/vector_map_center_lines_rviz"
 * @param Start_Position in simulation environment like rviz, this node require the user to select start position using "2D Pose Estimate" button and select starting position from the global path, //
 * if localization node is working and ndt_pose or curr_pose messages are published the node will use localization as starting position instead of "2D Pose Estimate"
 * @param Goal_Position destination to generate global plan to. if "replan" option selection used can choose multiple goal positions. goal positions are selected from Rviz using "2D Nav Goal" button.
 * @return global , reference path as list of waypoints. data type is "autoware_msgs::LaneArray" , and the topic name is "lane_waypoints_array"
 */

/**
 * @brief global Planner 表示全局规划模块，用于生成全局规划（参考路径），节点需要起始位置、目标位置和返回一系列航点
 * 
 * @brief 节点启动后（根据映射选项），它将加载矢量地图（路网）并将其发送到Rviz进行可视化。
 *        话题名为 "/vector_map_center_lines_rviz"
 * 
 * @param Start_Position 在仿真环境中，如Rviz，该节点要求用户使用 2D Pose Estimate 按钮选择起始位置，并从全局路径中选择起始位置
 *        如果定位节点工作正常，并且 ndt_pose 或 curr_pose 消息发布，则节点将使用定位作为起始位置，而不是 2D Pose Estimate
 * 
 * @param Goal_Position 生成全局规划的目标位置，如果使用了 "replan" 选项选择，则可以选择多个目标位置。目标位置是从 Rviz中使用 2D Nav Goal按钮选择
 * 
 * @return 全局，参考路径作为航电列表。数据类型是 autoware_msgs::LaneArray， 话题名称是 "lane_waypoints_array"
 */
#include <ros/ros.h>
#include "op_global_planner_core.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "op_global_planner");
  GlobalPlanningNS::GlobalPlanner global_planner;
  global_planner.MainLoop();
  return 0;
}
