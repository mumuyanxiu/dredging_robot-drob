// Copyright (c) 2025
#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <queue>
#include <unordered_map>
#include <vector>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner {

void CustomPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  tf_ = tf;
  node_ = parent.lock();
  name_ = std::move(name);
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.10));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".robot_radius", rclcpp::ParameterValue(0.30));
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".allow_unknown", rclcpp::ParameterValue(true));

  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
  node_->get_parameter(name_ + ".robot_radius", robot_radius_);
  node_->get_parameter(name_ + ".allow_unknown", allow_unknown_);

  RCLCPP_INFO(node_->get_logger(),
              "CustomPlanner configured. frame=%s res=%.3f radius=%.2f allow_unknown=%d",
              global_frame_.c_str(), costmap_->getResolution(), robot_radius_, allow_unknown_);
}

void CustomPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "清理 CustomPlanner: %s", name_.c_str());
}

void CustomPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "激活 CustomPlanner: %s", name_.c_str());
}

void CustomPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "停用 CustomPlanner: %s", name_.c_str());
}

nav_msgs::msg::Path CustomPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path path_msg;
  path_msg.header.stamp = node_->now();
  path_msg.header.frame_id = global_frame_;

  if (!costmap_) {
    throw nav2_core::PlannerException("Costmap is null in CustomPlanner");
  }

  unsigned int sx, sy, gx, gy;
  if (!worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
      !worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))
  {
    throw nav2_core::PlannerException("Start or goal out of costmap bounds");
  }

  if (!isCellFree(sx, sy) || !isCellFree(gx, gy)) {
    throw nav2_core::PlannerException("Start or goal cell is occupied");
  }

  std::vector<geometry_msgs::msg::PoseStamped> pts;
  if (!aStarPlan(sx, sy, gx, gy, pts)) {
    throw nav2_core::PlannerException("A* failed to find a path");
  }

  // 可选：对路径进行稀疏化/插值，这里直接输出栅格步进点
  for (auto & p : pts) {
    p.header = path_msg.header;
    path_msg.poses.push_back(p);
  }

  return path_msg;
}

bool CustomPlanner::worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const
{
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();

  if (wx < origin_x || wy < origin_y) {
    return false;
  }
  mx = static_cast<unsigned int>((wx - origin_x) / resolution);
  my = static_cast<unsigned int>((wy - origin_y) / resolution);
  return mx < costmap_->getSizeInCellsX() && my < costmap_->getSizeInCellsY();
}

void CustomPlanner::mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const
{
  double origin_x = costmap_->getOriginX();
  double origin_y = costmap_->getOriginY();
  double resolution = costmap_->getResolution();
  wx = origin_x + (mx + 0.5) * resolution;
  wy = origin_y + (my + 0.5) * resolution;
}

bool CustomPlanner::isCellFree(unsigned int mx, unsigned int my) const
{
  unsigned char c = costmap_->getCost(mx, my);
  if (c == nav2_costmap_2d::LETHAL_OBSTACLE) {
    return false;
  }
  if (!allow_unknown_ && c == nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }

  // 简单半径膨胀碰撞检查
  const double resolution = costmap_->getResolution();
  const int r_cells = static_cast<int>(std::ceil(robot_radius_ / resolution));
  const int max_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int max_y = static_cast<int>(costmap_->getSizeInCellsY());
  for (int dy = -r_cells; dy <= r_cells; ++dy) {
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      int nx = static_cast<int>(mx) + dx;
      int ny = static_cast<int>(my) + dy;
      if (nx < 0 || ny < 0 || nx >= max_x || ny >= max_y) {
        if (!allow_unknown_) {
          return false;
        }
        continue;
      }
      // 圆形近似
      if (dx * dx + dy * dy > r_cells * r_cells) {
        continue;
      }
      unsigned char cc = costmap_->getCost(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny));
      if (cc >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
        return false;
      }
      if (!allow_unknown_ && cc == nav2_costmap_2d::NO_INFORMATION) {
        return false;
      }
    }
  }
  return true;
}

double CustomPlanner::heuristic(int x1, int y1, int x2, int y2) const
{
  const double dx = static_cast<double>(x2 - x1);
  const double dy = static_cast<double>(y2 - y1);
  return std::hypot(dx, dy);
}

bool CustomPlanner::aStarPlan(
  unsigned int start_mx, unsigned int start_my,
  unsigned int goal_mx, unsigned int goal_my,
  std::vector<geometry_msgs::msg::PoseStamped> & out)
{
  struct Node { int x; int y; double g; double f; int parent_key; };
  auto key_of = [](int x, int y) { return (x << 16) ^ y; };

  const int sx = static_cast<int>(start_mx);
  const int sy = static_cast<int>(start_my);
  const int gx = static_cast<int>(goal_mx);
  const int gy = static_cast<int>(goal_my);

  auto cmp = [](const Node & a, const Node & b) { return a.f > b.f; };
  std::priority_queue<Node, std::vector<Node>, decltype(cmp)> open(cmp);
  std::unordered_map<int, Node> best;  // key -> node with best g

  Node start {sx, sy, 0.0, heuristic(sx, sy, gx, gy), -1};
  open.push(start);
  best.emplace(key_of(sx, sy), start);

  const int dirs[8][2] = {
    {1,0},{-1,0},{0,1},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}
  };

  const int max_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int max_y = static_cast<int>(costmap_->getSizeInCellsY());

  while (!open.empty()) {
    Node cur = open.top();
    open.pop();

    if (cur.x == gx && cur.y == gy) {
      // 回溯
      std::vector<Node> rev;
      int k = key_of(cur.x, cur.y);
      while (k != -1) {
        const Node & n = best.at(k);
        rev.push_back(n);
        k = n.parent_key;
      }
      // 转换为世界坐标
      out.clear();
      for (auto it = rev.rbegin(); it != rev.rend(); ++it) {
        double wx, wy;
        mapToWorld(static_cast<unsigned int>(it->x), static_cast<unsigned int>(it->y), wx, wy);
        geometry_msgs::msg::PoseStamped p;
        p.pose.position.x = wx;
        p.pose.position.y = wy;
        p.pose.position.z = 0.0;
        p.pose.orientation.w = 1.0;
        out.push_back(std::move(p));
      }
      return true;
    }

    for (auto & d : dirs) {
      int nx = cur.x + d[0];
      int ny = cur.y + d[1];
      if (nx < 0 || ny < 0 || nx >= max_x || ny >= max_y) {
        continue;
      }
      if (!isCellFree(static_cast<unsigned int>(nx), static_cast<unsigned int>(ny))) {
        continue;
      }
      const double step = (d[0] != 0 && d[1] != 0) ? std::sqrt(2.0) : 1.0;
      Node nxt {nx, ny, cur.g + step, 0.0, key_of(cur.x, cur.y)};
      nxt.f = nxt.g + heuristic(nx, ny, gx, gy);
      int nk = key_of(nx, ny);
      auto it = best.find(nk);
      if (it == best.end() || nxt.g < it->second.g) {
        best[nk] = nxt;
        open.push(nxt);
      }
    }
  }

  return false;
}

}  // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner, nav2_core::GlobalPlanner)