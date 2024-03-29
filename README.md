
编译
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select op_global_planner


op_global_planner BACKUP
https://paste.nugine.xyz/uufshu2q

---

# autoware_note

- [x] waypoint_saver
- [x] waypoint_extractor 
- [x] waypoint_loader
- [x] waypoint_replanner
- [x] waypoint_marker_publisher
- [x] lane_navi
- [x] lane_rule 
- [x] lane_stop
- [x] ndt_mapping
- [x] op_global_planner


TODO: 

- [ ] op_local_planner
- [ ] ndt_matching
- [ ] 对比ndt_mapping和ndt_matching 
- [ ] 对比ndt_matching | nat_mapping 和 simple_ndt_slam区别，增加内容
- [ ] 替换LIO-SAM
- [ ] 直接写一个一键启动runtimemanager的sh脚本，螺旋升天
- [ ] lane_select
- [ ] astar_avoid
- [ ] velocity_set

---

CWH:　cmd

脱离 runtime 导入pcd地图文件
```
roslaunch map_file points_map_loader.launch sceme_num:=noupdate parea_list:=None path_pcd:=/home/cwh/Desktop/sim_white/autoware-220302.pcd
```

导入vector　map文件
```
['rosrun', 'map_file', 'vector_map_loader', '/home/cwh/Desktop/sim_white/sim_white/dtlane.csv', '/home/cwh/Desktop/sim_white/sim_white/lane.csv', '/home/cwh/Desktop/sim_white/sim_white/line.csv', '/home/cwh/Desktop/sim_white/sim_white/node.csv', '/home/cwh/Desktop/sim_white/sim_white/point.csv', '/home/cwh/Desktop/sim_white/sim_white/whiteline.csv']
```


---

TODO:


[ INFO] [1679279706.208430329]: Received Goal Pose
[ INFO] [1679279706.209502506]: CWH Main Loop is false
Goal Found, LaneID: 11, Distance : 0, Angle: 0

Info: PlannerH -> Plan (A) Path With Size (17), MultiPaths No(1) Extraction Time :
New DP Path -> 32
[ INFO] [1679279706.248216545]: CWH Main Loop is true

再次发送，receive　到了
CWH Main Loop is true

页面里面的蓝色路径没有更新？　也有可能已经更新了　需要刷新一下rviz



通过2D Nav goal 发送的目标只能接收一次

```
[ INFO] [1679276743.239217322]: Received Goal Pose
Goal Found, LaneID: 10, Distance : 0, Angle: 0

Info: PlannerH -> Plan (A) Path With Size (14), MultiPaths No(1) Extraction Time :
New DP Path -> 25
```


**op_destinations_rviz　/ pub_GoalsListRviz**

```
markers: 
  - 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "HMI_Destinations"
    id: 0
    type: 9
    action: 0
    pose: 
      position: 
        x: 19.3986625671
        y: 9.86745834351
        z: 0.0
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.668461497866
        w: 0.743746748477
    scale: 
      x: 3.25
      y: 3.25
      z: 3.25
    color: 
      r: 1.0
      g: 0.0
      b: 0.0
      a: 0.899999976158
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: "G"
    mesh_resource: ''
    mesh_use_embedded_materials: False
```

**global_waypoints_rviz / pub_PathsRviz**

```
- 
    header: 
      seq: 0
      stamp: 
        secs: 0
        nsecs:         0
      frame_id: "map"
    ns: "global_velocity_lane_1"
    id: 24
    type: 9
    action: 0
    pose: 
      position: 
        x: 18.9637550053
        y: 13.5829206755
        z: 0.2
      orientation: 
        x: 0.0
        y: 0.0
        z: 0.0
        w: 0.0
    scale: 
      x: 0.0
      y: 0.0
      z: 0.0
    color: 
      r: 1.0
      g: 1.0
      b: 1.0
      a: 0.899999976158
    lifetime: 
      secs: 0
      nsecs:         0
    frame_locked: False
    points: []
    colors: []
    text: "20"
    mesh_resource: ''
    mesh_use_embedded_materials: False
```

---

TODO:

聚类： https://blog.csdn.net/Travis_X/article/details/115350840



---

TODO:

https://blog.csdn.net/weixin_42396991/article/details/117418973

https://blog.csdn.net/qq_39537898/article/details/118436448

https://github.com/autowarefoundation/autoware_ai/issues/795

https://answers.ros.org/question/390380/autoware-hatem-1130-object-avoidance-not-working/

https://answers.ros.org/question/357918/global-planning-with-autoware-114-and-osm-vector-map/


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


自动驾驶系列：激光雷达建图和定位(NDT) https://zhuanlan.zhihu.com/p/77623762

【Autoware】之ndt_mapping理论公式及代码对比 https://blog.csdn.net/qq_39537898/article/details/115439552

NDT（Normal Distributions Transform）算法原理与公式推导  https://www.cnblogs.com/21207-iHome/p/8039741.html

https://epsavlc.github.io/about.html

https://blog.csdn.net/weixin_44570248/article/details/118463105

lanelet
vectorMap区别

高精地图。

SLAM发展过程。
区别


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

6. Line
   - Line 表示线段，用 lid 作为识别编号；
   - 两个 Point 的连线即为 Line，bpid 和 fpid指的是线段的两个端点 Point 的识别编号；
   - blid 和 flid 表示线段之间的关联关系，分别是上一条和下一条 Line 的识别编号。
