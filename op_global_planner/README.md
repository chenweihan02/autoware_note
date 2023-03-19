# OpenPlanner - Global Planner 

## op_global_planner 

This node generate global path from start point to target point(s) on a map. Planning cost is distance only. the algorithm could be extended to handle other costs such as turning right and left busy lanes in the dynamic map. it supports autoware vector map, and special designed .kml maps.


这个节点从地图上的起点生成到目标点(们)的全局路径。规划成本只是距离。该算法可以扩展为处理动态地图中的其他成本，如右转和左转繁忙车道。它支持autoware向量地图和特殊设计的.kml地图。

### Outputs
Global path from start to goal, if multiple goals are set, replanning is automatic when the vehicle reaches the end one goal.

从起点到终点的全局路径，如果设置了多个目标，当车辆到达一个目标的末端时，自动重新规划。

### Options
Lane change is avilable (parralel lanes are detected automatically) 
Start/Goal(s) are set from Rviz, and saved to .csv files, if rviz param is disables, start/goal(s) will be loaded from .csv file at.

变道是可用的(平行车道会自动检测) 起点/终点(们)从Rviz设置，并保存到.csv文件，如果rviz参数被禁用，起点/终点(们)将从.csv文件加载。

### Requirements

1. vector map

### How to launch

* From a sourced terminal:

`roslaunch op_global_planner op_global_planner.launch`

* From Runtime Manager:

Computing Tab -> Mission Planning -> OpenPlanner - Global Planner  -> op_global_planner

### Subscriptions/Publications


```
Publications: 
 * /lane_waypoints_array [autoware_msgs::LaneArray]
 * /global_waypoints_rviz [visualization_msgs::MarkerArray]
 * /op_destinations_rviz [visualization_msgs::MarkerArray]
 * /vector_map_center_lines_rviz [visualization_msgs::MarkerArray]

Subscriptions: 
 * /initialpose [geometry_msgs::PoseWithCovarianceStamped]
 * /move_base_simple/goal [geometry_msgs::PoseStamped]
 * /current_pose [geometry_msgs::PoseStamped]
 * /current_velocity [geometry_msgs::TwistStamped]
 * /vector_map_info/* 
```

![Demo Movie](https://youtu.be/BS5nLtBsXPE)
