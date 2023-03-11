/**
 * 根据路由请求在矢量地图中寻找通往目的地的各条可行路径，并发布至话题"/lane_waypoints_array"
 */

#include <sstream>

#include <ros/console.h>
#include <tf/transform_datatypes.h>

#include <vector_map/vector_map.h>
#include "autoware_msgs/LaneArray.h"

#include "lane_planner/lane_planner_vmap.hpp"

namespace {

int waypoint_max;
double search_radius; // meter
double velocity; // km/h
std::string frame_id;
std::string output_file;

ros::Publisher waypoint_pub;

lane_planner::vmap::VectorMap all_vmap;
lane_planner::vmap::VectorMap lane_vmap;
tablet_socket_msgs::route_cmd cached_route;

std::vector<std::string> split(const std::string& str, char delim)
{
  std::stringstream ss(str);
  std::string s;
  std::vector<std::string> vec;
  while (std::getline(ss, s, delim))
    vec.push_back(s);

  if (!str.empty() && str.back() == delim)
    vec.push_back(std::string());

  return vec;
}

std::string join(const std::vector<std::string>& vec, char delim)
{
  std::string str;
  for (size_t i = 0; i < vec.size(); ++i) {
    str += vec[i];
    if (i != (vec.size() - 1))
      str += delim;
  }

  return str;
}

int count_lane(const lane_planner::vmap::VectorMap& vmap)
{
  int lcnt = -1;

  for (const vector_map::Lane& l : vmap.lanes) {
    if (l.lcnt > lcnt)
      lcnt = l.lcnt;
  }

  return lcnt;
}


// route_cmd 回调函数
void create_waypoint(const tablet_socket_msgs::route_cmd& msg)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id;

  if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty()) {
    cached_route.header = header;
    cached_route.point = msg.point;
    return;
  }

  /*
  coarse_vmap
    是根据传入create_coarse_vmap_from_route函数的 route_cmd msg构建而得的
    是只包含无人车预计要经过的Point数据的粗略地图
    其被当作一个粗略的导航用以在信息更完备的针对某个路网的矢量地图lane_vmap中找到
    对应的轨迹点Point和道路Lane构建更加完善的导航地图
  */
  lane_planner::vmap::VectorMap coarse_vmap = lane_planner::vmap::create_coarse_vmap_from_route(msg);
  if (coarse_vmap.points.size() < 2)
    return;

  std::vector<lane_planner::vmap::VectorMap> fine_vmaps;

  /*
    根据只有轨迹点的导航地图coarse_vmap
    从整个路网地图lane_vmap中搜寻信息
    构建信息完备的导航地图 fine_mostleft_vmap
    (包含轨迹点，道路，道路限速曲率等信息)
  */
  lane_planner::vmap::VectorMap fine_mostleft_vmap =
    lane_planner::vmap::create_fine_vmap(lane_vmap, lane_planner::vmap::LNO_MOSTLEFT, coarse_vmap,
                 search_radius, waypoint_max);
  if (fine_mostleft_vmap.points.size() < 2)
    return;
  fine_vmaps.push_back(fine_mostleft_vmap);


  // count_lane 函数返回传入函数的矢量地图
  // fine_mostleft_vmap中的最大车道数
  int lcnt = count_lane(fine_mostleft_vmap);

  // 下面的for循环补充完善fine_vmaps
  // 往里面添加以其他车道编号为基准寻找后续Lane的导航地图
  for (int i = lane_planner::vmap::LNO_MOSTLEFT + 1; i <= lcnt; ++i) {
    lane_planner::vmap::VectorMap v =
      lane_planner::vmap::create_fine_vmap(lane_vmap, i, coarse_vmap, search_radius, waypoint_max);
    if (v.points.size() < 2)
      continue;
    fine_vmaps.push_back(v);
  }

  // 下面从fine_vmaps中读取信息构建lane_waypoint
  autoware_msgs::LaneArray lane_waypoint;
  for (const lane_planner::vmap::VectorMap& v : fine_vmaps) {
    autoware_msgs::Lane l;
    l.header = header;
    l.increment = 1;

    size_t last_index = v.points.size() - 1;
    for (size_t i = 0; i < v.points.size(); ++i) {
      double yaw;
      if (i == last_index) {
        geometry_msgs::Point p1 =
          lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
        geometry_msgs::Point p2 =
          lane_planner::vmap::create_geometry_msgs_point(v.points[i - 1]);
        yaw = atan2(p2.y - p1.y, p2.x - p1.x);
        yaw -= M_PI;
      } else {
        geometry_msgs::Point p1 =
          lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
        geometry_msgs::Point p2 =
          lane_planner::vmap::create_geometry_msgs_point(v.points[i + 1]);
        yaw = atan2(p2.y - p1.y, p2.x - p1.x);
      }

      autoware_msgs::Waypoint w;
      w.pose.header = header;
      w.pose.pose.position = lane_planner::vmap::create_geometry_msgs_point(v.points[i]);
      w.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      w.twist.header = header;
      w.twist.twist.linear.x = velocity / 3.6; // to m/s
      l.waypoints.push_back(w);
    }
    lane_waypoint.lanes.push_back(l);
  }

  // 在话题 "/lane_waypoints_array"发布lane_waypoint
  waypoint_pub.publish(lane_waypoint);

  for (size_t i = 0; i < fine_vmaps.size(); ++i) {
    std::stringstream ss;
    ss << "_" << i;

    std::vector<std::string> v1 = split(output_file, '/');
    std::vector<std::string> v2 = split(v1.back(), '.');
    v2[0] = v2.front() + ss.str();
    v1[v1.size() - 1] = join(v2, '.');
    std::string path = join(v1, '/');

    lane_planner::vmap::write_waypoints(fine_vmaps[i].points, velocity, path);
  }
}

// 用以在接收到数据时更新 lane_vmap，当然要保证 all_vmap 中的 points，lanes 和 nodes 数据皆完备
// 才可以根据 all_vmap 去生成新的 lane_vmap。
// 接着如果接收到了路径规划的数据(cached_route.point 存在数据)，则根据 cached_route 创建导航轨迹点。
// 随后清除已被使用的 cached_route.point 数据。
void update_values()
{
  if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty())
    return;

  // lane_vmap 是 VectorMap 类型数据，与 all_vmap 一致
  // 并且 lane_planner::vmap::LNO_ALL = -1 
  lane_vmap = lane_planner::vmap::create_lane_vmap(all_vmap, lane_planner::vmap::LNO_ALL);

  // cached_route 为 route_cmd 类型数据
  if (!cached_route.point.empty()) {
    create_waypoint(cached_route);
    cached_route.point.clear();
    cached_route.point.shrink_to_fit();
  }
}

void cache_point(const vector_map::PointArray& msg)
{
  all_vmap.points = msg.data;
  update_values();
}

void cache_lane(const vector_map::LaneArray& msg)
{
  all_vmap.lanes = msg.data;
  update_values();
}

void cache_node(const vector_map::NodeArray& msg)
{
  all_vmap.nodes = msg.data;
  update_values();
}

} // namespace

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_navi");

  ros::NodeHandle n;

  int sub_vmap_queue_size;
  n.param<int>("/lane_navi/sub_vmap_queue_size", sub_vmap_queue_size, 1);
  int sub_route_queue_size;
  n.param<int>("/lane_navi/sub_route_queue_size", sub_route_queue_size, 1);
  int pub_waypoint_queue_size;
  n.param<int>("/lane_navi/pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
  bool pub_waypoint_latch;
  n.param<bool>("/lane_navi/pub_waypoint_latch", pub_waypoint_latch, true);

  n.param<int>("/lane_navi/waypoint_max", waypoint_max, 10000);
  n.param<double>("/lane_navi/search_radius", search_radius, 10);
  n.param<double>("/lane_navi/velocity", velocity, 40);
  n.param<std::string>("/lane_navi/frame_id", frame_id, "map");
  n.param<std::string>("/lane_navi/output_file", output_file, "/tmp/lane_waypoint.csv");

  if (output_file.empty()) {
    ROS_ERROR_STREAM("output filename is empty");
    return EXIT_FAILURE;
  }
  if (output_file.back() == '/') {
    ROS_ERROR_STREAM(output_file << " is directory");
    return EXIT_FAILURE;
  }

  waypoint_pub = n.advertise<autoware_msgs::LaneArray>("/lane_waypoints_array", pub_waypoint_queue_size,
                 pub_waypoint_latch);

  ros::Subscriber route_sub = n.subscribe("/route_cmd", sub_route_queue_size, create_waypoint);
  ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
  ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
  ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);

  ros::spin();

  return EXIT_SUCCESS;
}
