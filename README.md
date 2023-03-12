# autoware_note

- [x] waypoint_saver
- [x] waypoint_extractor 
- [x] waypoint_loader
- [x] waypoint_replanner
- [x] waypoint_marker_publisher
- [x] lane_navi

TODO: 
- [ ] lane_rule


---

TODO: 

lane_navi可以理解为车道线规划（车道线级别的路径规划），主要作用是根据车道线信息
以及上层（AutowareTouch.apk）下发的航迹点（route_cmd）来进行规划。
获得经过设定航迹点（可获得多条不同车道上的路径）
！！！需要手动实现 通过lane_navi发送route_cmd消息、


lane_navi节点会订阅/waypoint_saver/loaded_waypoints话题，并在回调函数中更新cached_route变量。
每次收到新的航迹点时，它会清空cached_route变量，并将新的航迹点序列添加到cached_route变量中。

然后，lane_navi节点会根据当前车辆的位置和方向，在cached_route变量中找到最近的一条航迹点序列，
并将其发布到/final_waypoints话题上。
这个话题是由astar_avoid或者waypoint_follower节点订阅的，它们会根据这些航迹点进行规划和控制。


---

TODO:

自动驾驶实战系列(一)——构建点云地图的关键技术：http://49.235.237.45/2019/10/11/31ndt-map/
Autoware planning模块学习笔记（二）：路径规划（4）- 节点lane_navi：https://blog.csdn.net/xiaoxiao123jun/article/details/105227743

vector_map矢量化地图的生成方法 https://blog.csdn.net/qq_35635374/article/details/120920983 

---
---

## vector_map

1. Point
   - 存储了地图中所有点元素的几何位置属性，包括路径节点node、路径特征补充节点dtlane、线状面状端点
   - Point点为属性变更点或者多条lane规划线的链接节点

2. Node
   - Node是地图生成车道lane的元素，是车道lane元素的节点
   - node节点相连形成了车道lane。在绘制地图时，绘制一个node后会在point.csv中存储几何及位置信息
   - node.csv与point.csv通过PID相链接

3. DTLane
   - DTLane 为道路或车道中心线添加了线性元素和纵向横向坡度的数据，承担着中心线性数据的作用
   - Dtlane是对车道元素lane的几个形状特征的补充元素
   - 在绘制地图时，绘制一个Dtlane后会在point.csv中存储几何及位置信息

4. Lane 
   - 定义车辆行驶的车道，它由两个Node组成。每段lane的长度即是before node和forward node之间的距离
   - 地图中的lane代表小车形式的轨迹路径，一般是道路的中心线，在显示道路中是没有的，是靠人为规划出来的

1. Line
   - Line 表示线段，用 lid 作为识别编号；
   - 两个 Point 的连线即为 Line，bpid 和 fpid指的是线段的两个端点 Point 的识别编号；
   - blid 和 flid 表示线段之间的关联关系，分别是上一条和下一条 Line 的识别编号。
