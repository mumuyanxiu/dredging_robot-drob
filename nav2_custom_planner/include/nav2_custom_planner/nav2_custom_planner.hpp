// Copyright (c) 2025
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace nav2_custom_planner {

class CustomPlanner : public nav2_core::GlobalPlanner {
public:
  CustomPlanner() = default;
  ~CustomPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

private:
  bool worldToMap(double wx, double wy, unsigned int & mx, unsigned int & my) const;
  void mapToWorld(unsigned int mx, unsigned int my, double & wx, double & wy) const;
  bool isCellFree(unsigned int mx, unsigned int my) const;
  double heuristic(int x1, int y1, int x2, int y2) const;
  bool aStarPlan(unsigned int start_mx, unsigned int start_my,
                 unsigned int goal_mx, unsigned int goal_my,
                 std::vector<geometry_msgs::msg::PoseStamped> & out);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  nav2_costmap_2d::Costmap2D * costmap_ {nullptr};
  std::string global_frame_;
  std::string name_;

  // 参数
  double interpolation_resolution_ {0.1};
  double robot_radius_ {0.30};
  bool allow_unknown_ {true};
};

}  // namespace nav2_custom_planner