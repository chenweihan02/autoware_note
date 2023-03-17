/**
 * 对节点 lane_navi 发布在话题“/lane_waypoints_array”上的规划路径在轨迹点速度方面进一步修正，为红灯时在停车线内减速停车等场景提供支持
 */

// #define DEBUG

#ifdef DEBUG
#include <sstream>
#endif  // DEBUG

#include <ros/console.h>

#include <vector_map/vector_map.h>
#include "autoware_config_msgs/ConfigLaneRule.h"
#include "autoware_msgs/LaneArray.h"

#include <lane_planner/lane_planner_vmap.hpp>

namespace
{
double config_acceleration = 1;            // m/s^2
double config_stopline_search_radius = 1;  // meter
int config_number_of_zeros_ahead = 0;
int config_number_of_zeros_behind = 0;
int config_number_of_smoothing_count = 0;

int waypoint_max;
double search_radius;  // meter
double curve_weight;
double crossroad_weight;
double clothoid_weight;
std::string frame_id;

ros::Publisher traffic_pub;
ros::Publisher red_pub;
ros::Publisher green_pub;

lane_planner::vmap::VectorMap all_vmap;
lane_planner::vmap::VectorMap lane_vmap;
double curve_radius_min;
double crossroad_radius_min;
double clothoid_radius_min;
autoware_msgs::LaneArray cached_waypoint;

#ifdef DEBUG
visualization_msgs::Marker debug_marker;
ros::Publisher marker_pub;
int marker_cnt;
#endif  // DEBUG

autoware_msgs::Lane create_new_lane(const autoware_msgs::Lane& lane, const std_msgs::Header& header)
{
  autoware_msgs::Lane l = lane;
  l.header = header;

  for (autoware_msgs::Waypoint& w : l.waypoints)
  {
    w.pose.header = header;
    w.twist.header = header;
  }

  return l;
}

autoware_msgs::Lane apply_acceleration(const autoware_msgs::Lane& lane, double acceleration, size_t start_index,
                                       size_t fixed_cnt, double fixed_vel)
{
  autoware_msgs::Lane l = lane;

  if (fixed_cnt == 0)
    return l;

  double square_vel = fixed_vel * fixed_vel;
  double distance = 0;
  for (size_t i = start_index; i < l.waypoints.size(); ++i)
  {
    if (i - start_index < fixed_cnt)
    {
      l.waypoints[i].twist.twist.linear.x = fixed_vel;
      continue;
    }

    geometry_msgs::Point a = l.waypoints[i - 1].pose.pose.position;
    geometry_msgs::Point b = l.waypoints[i].pose.pose.position;
    distance += hypot(b.x - a.x, b.y - a.y);

    const int sgn = (l.waypoints[i].twist.twist.linear.x < 0.0) ? -1 : 1;
    double v = sqrt(square_vel + 2 * acceleration * distance);
    if (v < l.waypoints[i].twist.twist.linear.x)
    {
      l.waypoints[i].twist.twist.linear.x = sgn * v;
    }
    else
    {
      break;
    }
  }

  return l;
}

autoware_msgs::Lane apply_crossroad_acceleration(const autoware_msgs::Lane& lane, double acceleration)
{
  autoware_msgs::Lane l = lane;

  bool crossroad = false;
  std::vector<size_t> start_indexes;
  std::vector<size_t> end_indexes;
  for (size_t i = 0; i < l.waypoints.size(); ++i)
  {
    vector_map::DTLane dtlane = lane_planner::vmap::create_vector_map_dtlane(l.waypoints[i].dtlane);
    if (i == 0)
    {
      crossroad = lane_planner::vmap::is_crossroad_dtlane(dtlane);
      continue;
    }
    if (crossroad)
    {
      if (!lane_planner::vmap::is_crossroad_dtlane(dtlane))
      {
        end_indexes.push_back(i - 1);
        crossroad = false;
      }
    }
    else
    {
      if (lane_planner::vmap::is_crossroad_dtlane(dtlane))
      {
        start_indexes.push_back(i);
        crossroad = true;
      }
    }
  }
  if (start_indexes.empty() && end_indexes.empty())
    return l;

  for (const size_t i : end_indexes)
    l = apply_acceleration(l, acceleration, i, 1, l.waypoints[i].twist.twist.linear.x);

  std::reverse(l.waypoints.begin(), l.waypoints.end());

  std::vector<size_t> reverse_start_indexes;
  for (const size_t i : start_indexes)
    reverse_start_indexes.push_back(l.waypoints.size() - i - 1);
  std::reverse(reverse_start_indexes.begin(), reverse_start_indexes.end());

  for (const size_t i : reverse_start_indexes)
    l = apply_acceleration(l, acceleration, i, 1, l.waypoints[i].twist.twist.linear.x);

  std::reverse(l.waypoints.begin(), l.waypoints.end());

  return l;
}

autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane, double acceleration,
                                                const lane_planner::vmap::VectorMap& fine_vmap, size_t ahead_cnt,
                                                size_t behind_cnt)
{
  autoware_msgs::Lane l = lane;

  std::vector<size_t> indexes;
  for (size_t i = 0; i < fine_vmap.stoplines.size(); ++i)
  {
    if (fine_vmap.stoplines[i].id >= 0)
      indexes.push_back(i);
  }
  if (indexes.empty())
    return l;

  for (const size_t i : indexes)
    l = apply_acceleration(l, acceleration, i, behind_cnt + 1, 0);

  std::reverse(l.waypoints.begin(), l.waypoints.end());

  std::vector<size_t> reverse_indexes;
  for (const size_t i : indexes)
    reverse_indexes.push_back(l.waypoints.size() - i - 1);
  std::reverse(reverse_indexes.begin(), reverse_indexes.end());

  for (const size_t i : reverse_indexes)
    l = apply_acceleration(l, acceleration, i, ahead_cnt + 1, 0);

  std::reverse(l.waypoints.begin(), l.waypoints.end());

  return l;
}

std::vector<vector_map::Point> create_stop_points(const lane_planner::vmap::VectorMap& vmap)
{
  std::vector<vector_map::Point> stop_points;
  for (const vector_map::StopLine& s : vmap.stoplines)
  {
    for (const vector_map::Lane& l : vmap.lanes)
    {
      if (l.lnid != s.linkid)
        continue;
      for (const vector_map::Node& n : vmap.nodes)
      {
        if (n.nid != l.bnid)
          continue;
        for (const vector_map::Point& p : vmap.points)
        {
          if (p.pid != n.pid)
            continue;
          bool hit = false;
          for (const vector_map::Point& sp : stop_points)
          {
            if (sp.pid == p.pid)
            {
              hit = true;
              break;
            }
          }
          if (!hit)
            stop_points.push_back(p);
        }
      }
    }
  }

  return stop_points;
}

std::vector<size_t> create_stop_indexes(const lane_planner::vmap::VectorMap& vmap, const autoware_msgs::Lane& lane,
                                        double stopline_search_radius)
{
  std::vector<size_t> stop_indexes;
  for (const vector_map::Point& p : create_stop_points(vmap))
  {
    size_t index = SIZE_MAX;
    double distance = DBL_MAX;
    for (size_t i = 0; i < lane.waypoints.size(); ++i)
    {
      vector_map::Point point = lane_planner::vmap::create_vector_map_point(lane.waypoints[i].pose.pose.position);
      double d = hypot(p.bx - point.bx, p.ly - point.ly);
      if (d <= distance)
      {
        index = i;
        distance = d;
      }
    }
    if (index != SIZE_MAX && distance <= stopline_search_radius)
    {
      stop_indexes.push_back(index);
    }
  }
  std::sort(stop_indexes.begin(), stop_indexes.end());

  return stop_indexes;
}

// 对车速进行修正。
autoware_msgs::Lane apply_stopline_acceleration(const autoware_msgs::Lane& lane, double acceleration,
                                                double stopline_search_radius, size_t ahead_cnt, size_t behind_cnt)
{
  autoware_msgs::Lane l = lane;

  // 查找l中所有停止点（停止线跟前的轨迹点）的下标
  std::vector<size_t> indexes = create_stop_indexes(lane_vmap, l, stopline_search_radius);
  if (indexes.empty())
    return l;

  for (const size_t i : indexes)
    // 修正l中轨迹点的速度，首先固定l内下标在 [i, i + behind_cnt + 1)范围内的轨迹点速度都为0
    // 随后根据加速情况下的运动学公式修正后面轨迹点的速度
    l = apply_acceleration(l, acceleration, i, behind_cnt + 1, 0);
  // 反转 l.waypoints中的所有元素
  std::reverse(l.waypoints.begin(), l.waypoints.end());

  std::vector<size_t> reverse_indexes;
  for (const size_t i : indexes)
    // 记录反转后的l.waypoints中的停止点的下标，此时下标是从大到小
    reverse_indexes.push_back(l.waypoints.size() - i - 1);
  std::reverse(reverse_indexes.begin(), reverse_indexes.end());

  for (const size_t i : reverse_indexes)
  // 反转后再调用 apply_acceleration
  // 总体而言，这里与上面调用 apply_acceleration 函数的情况相反 
  // 此处对于停止点和还未到停止点前的一些轨迹点令车静止
  // 而这些轨迹点更前面的轨迹点根据减速到达停止点修正原车速

    l = apply_acceleration(l, acceleration, i, ahead_cnt + 1, 0);
  // 上面两次调用 apply_acceleration 函数
  // 对车速进行修正，首先以停车点为中心修正附近轨迹点车速为 0
  // 接着运用匀减速/匀加速动力学方程修正
  // 匀减速接近/匀加速离开停车点过程中的车速


  // 恢复 l.waypoints 中轨迹点的顺序
  std::reverse(l.waypoints.begin(), l.waypoints.end());

  return l;
}

bool is_fine_vmap(const lane_planner::vmap::VectorMap& fine_vmap, const autoware_msgs::Lane& lane)
{
  if (fine_vmap.points.size() != lane.waypoints.size())
    return false;

  for (size_t i = 0; i < fine_vmap.points.size(); ++i)
  {
    // 要先用create_vector_map_point函数进行转换
    // 因为矢量地图内轨迹点坐标与geometry_msgs::Point类型的轨迹点x-y坐标是反向的
    vector_map::Point point = lane_planner::vmap::create_vector_map_point(lane.waypoints[i].pose.pose.position);
    double distance = hypot(fine_vmap.points[i].bx - point.bx, fine_vmap.points[i].ly - point.ly);
    // 对应的轨迹点应当是在同一位置
    if (distance > 0.1)
      return false;
  }

  return true;
}

double create_reduction(const lane_planner::vmap::VectorMap& fine_vmap, int index)
{
  const vector_map::DTLane& dtlane = fine_vmap.dtlanes[index];

  if (lane_planner::vmap::is_straight_dtlane(dtlane))
    return 1;

  if (lane_planner::vmap::is_curve_dtlane(dtlane))
  {
    if (lane_planner::vmap::is_crossroad_dtlane(dtlane))
      return lane_planner::vmap::compute_reduction(dtlane, crossroad_radius_min * crossroad_weight);

    if (lane_planner::vmap::is_connection_dtlane(fine_vmap, index))
      return 1;

    return lane_planner::vmap::compute_reduction(dtlane, curve_radius_min * curve_weight);
  }

  if (lane_planner::vmap::is_clothoid_dtlane(dtlane))
    return lane_planner::vmap::compute_reduction(dtlane, clothoid_radius_min * clothoid_weight);

  return 1;
}

#ifdef DEBUG
std_msgs::ColorRGBA create_color(int index)
{
  std_msgs::ColorRGBA color;
  switch (index)
  {
    case 0:
      color.r = 0;
      color.g = 0;
      color.b = 0;
      break;
    case 1:
      color.r = 0;
      color.g = 0;
      color.b = 1;
      break;
    case 2:
      color.r = 0;
      color.g = 1;
      color.b = 0;
      break;
    case 3:
      color.r = 0;
      color.g = 1;
      color.b = 1;
      break;
    case 4:
      color.r = 1;
      color.g = 0;
      color.b = 0;
      break;
    case 5:
      color.r = 1;
      color.g = 0;
      color.b = 1;
      break;
    case 6:
      color.r = 1;
      color.g = 1;
      color.b = 0;
      break;
    default:
      color.r = 1;
      color.g = 1;
      color.b = 1;
  }
  color.a = 1;

  return color;
}
#endif  // DEBUG

// waypoint_sub 回调函数  config_parameter  update_values调用
/*
lane_rule: create_waypoint接收的是autoware_msgs::LaneArray
lane_navi: create_waypoint接收的是table_socket_msgs::route_cmd

lane_rule：waypoint_sub的回调函数，话题是 "lane_waypoints_array"
该话题的发布者在lane_navi waypoint_pub

lane_navi根据路由指令在矢量地图内搜索到达目的地的路径并在话题 lane_waypoints_array上发布
被节点lane_rule监听后对路径进行修正

*/
void create_waypoint(const autoware_msgs::LaneArray& msg)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = frame_id;

  cached_waypoint.lanes.clear();
  cached_waypoint.lanes.shrink_to_fit();
  cached_waypoint.id = msg.id;
  for (const autoware_msgs::Lane& l : msg.lanes)
    cached_waypoint.lanes.push_back(create_new_lane(l, header));

  if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty() || all_vmap.stoplines.empty() ||
      all_vmap.dtlanes.empty())
  {
    traffic_pub.publish(cached_waypoint);
    return;
  }

#ifdef DEBUG
  marker_cnt = msg.lanes.size();
#endif  // DEBUG

  autoware_msgs::LaneArray traffic_waypoint;
  autoware_msgs::LaneArray red_waypoint;
  autoware_msgs::LaneArray green_waypoint;
  traffic_waypoint.id = red_waypoint.id = green_waypoint.id = msg.id;
  for (size_t i = 0; i < msg.lanes.size(); ++i)
  {
    autoware_msgs::Lane lane = create_new_lane(msg.lanes[i], header);
    // 创建只有轨迹点的粗略导航地图
    lane_planner::vmap::VectorMap coarse_vmap = lane_planner::vmap::create_coarse_vmap_from_lane(lane);
    if (coarse_vmap.points.size() < 2)
    {
      traffic_waypoint.lanes.push_back(lane);
      continue;
    }

    lane_planner::vmap::VectorMap fine_vmap = lane_planner::vmap::create_fine_vmap(
        lane_vmap, lane_planner::vmap::LNO_ALL, coarse_vmap, search_radius, waypoint_max);
    
    // is_fine_vmap函数根据lane中的数据对比fine_vmap中
    // 储存的数据 判断fine_vmap是否正确生成
    if (fine_vmap.points.size() < 2 || !is_fine_vmap(fine_vmap, lane))
    {
      traffic_waypoint.lanes.push_back(lane);
      green_waypoint.lanes.push_back(lane);
      // apply_stopline_acceleration 函数对传入函数的
      // lane里面接近停止点和离开停止点的轨迹点速度进行修正
      lane = apply_stopline_acceleration(lane, config_acceleration, config_stopline_search_radius,
                                         config_number_of_zeros_ahead, config_number_of_zeros_behind);
      red_waypoint.lanes.push_back(lane);
      continue;
    }

    for (size_t j = 0; j < lane.waypoints.size(); ++j)
    {
      // 根据当前轨迹点所处的lane 是否为弯道或者回旋道等情况降低车速
      lane.waypoints[j].twist.twist.linear.x *= create_reduction(fine_vmap, j);
      if (fine_vmap.dtlanes[j].did >= 0)
      {
        lane.waypoints[j].dtlane = lane_planner::vmap::create_waypoint_follower_dtlane(fine_vmap.dtlanes[j]);
      }
    }

    /* velocity smoothing */
    // 光滑速度变化
    for (int k = 0; k < config_number_of_smoothing_count; ++k)
    {
      autoware_msgs::Lane temp_lane = lane;
      // 检查轨迹数目
      if (lane.waypoints.size() >= 3)
      {
        // 遍历lane中的轨迹点
        for (size_t j = 1; j < lane.waypoints.size() - 1; ++j)
        {
          if (lane.waypoints.at(j).twist.twist.linear.x != 0)
          {
            // 顺滑速度的方式：以下标为j的轨迹点为中心
            // 将三个连续点速度取均值作为下标为j的轨迹点的速度
            lane.waypoints[j].twist.twist.linear.x =
                (temp_lane.waypoints.at(j - 1).twist.twist.linear.x + temp_lane.waypoints.at(j).twist.twist.linear.x +
                 temp_lane.waypoints.at(j + 1).twist.twist.linear.x) /
                3;
          }
        }
      }
    }

    // apply_crossroad_acceleration 函数对传入的函数的lane里面
    // 接近交叉口起始点和离开交叉口末尾点的轨迹点速度进行修正
    lane = apply_crossroad_acceleration(lane, config_acceleration);

    traffic_waypoint.lanes.push_back(lane);
    green_waypoint.lanes.push_back(lane);

    // 此处 apply_crossroad_acceleration 函数与前面的 为同名函数
    // 前面的 apply_crossroad_acceleration 函数因为缺少导航矢量地图 fine_vmap
    // 需要在全局路网地图 lane_vmap中寻找停止点，
    // 而此处的 apply_crossroad_acceleration 函数直接借助 fine_vmap即可找到行车路径上的停止点
    lane = apply_stopline_acceleration(lane, config_acceleration, fine_vmap, config_number_of_zeros_ahead,
                                       config_number_of_zeros_behind);

    red_waypoint.lanes.push_back(lane);

#ifdef DEBUG
    std::stringstream ss;
    ss << "_" << i;

    visualization_msgs::Marker m = debug_marker;
    m.ns = "lane" + ss.str();
    m.color = create_color(i);

    lane_planner::vmap::publish_add_marker(marker_pub, m, fine_vmap.points);
#endif  // DEBUG
  }

  traffic_pub.publish(traffic_waypoint);
  red_pub.publish(red_waypoint);
  // rad_waypoint 是经过 apply_crossroad_acceleration 函数处理过的
  // 应用于红灯时在停车线减速停车的场景
  green_pub.publish(green_waypoint);
}

/*
相比于lane_navi多了对dtlanes和 stopline的判断
*/
void update_values()
{
  if (all_vmap.points.empty() || all_vmap.lanes.empty() || all_vmap.nodes.empty() || all_vmap.stoplines.empty() ||
      all_vmap.dtlanes.empty())
    return;

  lane_vmap = lane_planner::vmap::create_lane_vmap(all_vmap, lane_planner::vmap::LNO_ALL);

  curve_radius_min = lane_planner::vmap::RADIUS_MAX;
  crossroad_radius_min = lane_planner::vmap::RADIUS_MAX;
  clothoid_radius_min = lane_planner::vmap::RADIUS_MAX;

  // 

  for (const vector_map::DTLane& d : lane_vmap.dtlanes)
  {
    double radius_min = fabs(d.r);
    // 弯道
    if (lane_planner::vmap::is_curve_dtlane(d))
    {
      // 十字路口
      if (lane_planner::vmap::is_crossroad_dtlane(d))
      {
        if (radius_min < crossroad_radius_min)
          crossroad_radius_min = radius_min;
      }
      else
      {
        if (radius_min < curve_radius_min)
          curve_radius_min = radius_min;
      }
    }
    // 道路回旋
    else if (lane_planner::vmap::is_clothoid_dtlane(d))
    {
      if (radius_min < clothoid_radius_min)
        clothoid_radius_min = radius_min;
    }
  }

#ifdef DEBUG
  for (int i = 0; i < marker_cnt; ++i)
  {
    std::stringstream ss;
    ss << "_" << i;

    visualization_msgs::Marker m = debug_marker;
    m.ns = "lane" + ss.str();

    lane_planner::vmap::publish_delete_marker(marker_pub, m);
  }
  marker_cnt = 0;
#endif  // DEBUG

  if (!cached_waypoint.lanes.empty())
  {
    autoware_msgs::LaneArray update_waypoint = cached_waypoint;
    create_waypoint(update_waypoint);
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

void cache_stopline(const vector_map::StopLineArray& msg)
{
  all_vmap.stoplines = msg.data;
  update_values();
}

void cache_dtlane(const vector_map::DTLaneArray& msg)
{
  all_vmap.dtlanes = msg.data;
  update_values();
}

void config_parameter(const autoware_config_msgs::ConfigLaneRule& msg)
{
  config_acceleration = msg.acceleration;
  config_stopline_search_radius = msg.stopline_search_radius;
  config_number_of_zeros_ahead = msg.number_of_zeros_ahead;
  config_number_of_zeros_behind = msg.number_of_zeros_behind;
  config_number_of_smoothing_count = msg.number_of_smoothing_count;

  if (!cached_waypoint.lanes.empty())
  {
    autoware_msgs::LaneArray update_waypoint = cached_waypoint;
    create_waypoint(update_waypoint);
  }
}

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lane_rule");

  ros::NodeHandle n;
  ros::NodeHandle pnh("~");

  int sub_vmap_queue_size;
  pnh.param<int>("sub_vmap_queue_size", sub_vmap_queue_size, 1);
  int sub_waypoint_queue_size;
  pnh.param<int>("sub_waypoint_queue_size", sub_waypoint_queue_size, 1);
  int sub_config_queue_size;
  pnh.param<int>("sub_config_queue_size", sub_config_queue_size, 1);
  int pub_waypoint_queue_size;
  pnh.param<int>("pub_waypoint_queue_size", pub_waypoint_queue_size, 1);
  bool pub_waypoint_latch;
  pnh.param<bool>("pub_waypoint_latch", pub_waypoint_latch, true);
#ifdef DEBUG
  int pub_marker_queue_size;
  pnh.param<int>("pub_marker_queue_size", pub_marker_queue_size, 10);
  bool pub_marker_latch;
  pnh.param<bool>("pub_marker_latch", pub_marker_latch, true);
#endif  // DEBUG

  pnh.param<int>("waypoint_max", waypoint_max, 10000);
  pnh.param<double>("search_radius", search_radius, 10);
  pnh.param<double>("curve_weight", curve_weight, 0.6);
  pnh.param<double>("crossroad_weight", crossroad_weight, 0.9);
  pnh.param<double>("clothoid_weight", clothoid_weight, 0.215);
  pnh.param<std::string>("frame_id", frame_id, "map");
  pnh.param<double>("acceleration", config_acceleration, 1);
  pnh.param<int>("number_of_smoothing_count", config_number_of_smoothing_count, 0);
  pnh.param<int>("number_of_zeros_ahead", config_number_of_zeros_ahead, 0);
  pnh.param<int>("number_of_zeros_behind", config_number_of_zeros_behind, 0);
  pnh.param<double>("stopline_search_radius", config_stopline_search_radius, 1);

  traffic_pub =
      n.advertise<autoware_msgs::LaneArray>("/traffic_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
  red_pub = n.advertise<autoware_msgs::LaneArray>("/red_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);
  green_pub =
      n.advertise<autoware_msgs::LaneArray>("/green_waypoints_array", pub_waypoint_queue_size, pub_waypoint_latch);

#ifdef DEBUG
  debug_marker.header.frame_id = frame_id;
  debug_marker.id = 0;
  debug_marker.type = visualization_msgs::Marker::LINE_STRIP;
  debug_marker.scale.x = 0.2;
  debug_marker.scale.y = 0.2;
  debug_marker.frame_locked = true;

  marker_pub = n.advertise<visualization_msgs::Marker>("/waypoint_debug", pub_marker_queue_size, pub_marker_latch);
#endif  // DEBUG

  ros::Subscriber waypoint_sub = n.subscribe("/lane_waypoints_array", sub_waypoint_queue_size, create_waypoint);
  ros::Subscriber point_sub = n.subscribe("/vector_map_info/point", sub_vmap_queue_size, cache_point);
  ros::Subscriber lane_sub = n.subscribe("/vector_map_info/lane", sub_vmap_queue_size, cache_lane);
  ros::Subscriber node_sub = n.subscribe("/vector_map_info/node", sub_vmap_queue_size, cache_node);
  ros::Subscriber stopline_sub = n.subscribe("/vector_map_info/stop_line", sub_vmap_queue_size, cache_stopline);
  ros::Subscriber dtlane_sub = n.subscribe("/vector_map_info/dtlane", sub_vmap_queue_size, cache_dtlane);
  ros::Subscriber config_sub = n.subscribe("/config/lane_rule", sub_config_queue_size, config_parameter);

  ros::spin();

  return EXIT_SUCCESS;
}
