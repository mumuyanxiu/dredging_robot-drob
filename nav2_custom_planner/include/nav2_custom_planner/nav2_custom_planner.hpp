#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner
{

    void CustomPlanner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent, std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        tf_ = tf;
        node_ = parent.lock();
        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();
        // 参数初始化
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(0.1));
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".coverage_resolution", rclcpp::ParameterValue(0.5));
        nav2_util::declare_parameter_if_not_declared(
            node_, name_ + ".start_corner", rclcpp::ParameterValue("bottom_left"));
        
        node_->get_parameter(name_ + ".interpolation_resolution",
                             interpolation_resolution_);
        node_->get_parameter(name_ + ".coverage_resolution",
                             coverage_resolution_);
        node_->get_parameter(name_ + ".start_corner",
                             start_corner_);
    }

    void CustomPlanner::cleanup()
    {
        RCLCPP_INFO(node_->get_logger(), "正在清理类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::activate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在激活类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    void CustomPlanner::deactivate()
    {
        RCLCPP_INFO(node_->get_logger(), "正在停用类型为 CustomPlanner 的插件 %s",
                    name_.c_str());
    }

    nav_msgs::msg::Path
    CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                              const geometry_msgs::msg::PoseStamped &goal)
    {
        // 1.声明并初始化 global_path
        nav_msgs::msg::Path global_path;
        global_path.poses.clear();
        global_path.header.stamp = node_->now();
        global_path.header.frame_id = global_frame_;

        // 2.获取代价地图的边界信息
        unsigned int map_width = costmap_->getSizeInCellsX();
        unsigned int map_height = costmap_->getSizeInCellsY();
        double map_resolution = costmap_->getResolution();
        
        // 获取地图的世界坐标边界
        double world_x_min, world_y_min, world_x_max, world_y_max;
        costmap_->mapToWorld(0, 0, world_x_min, world_y_min);
        costmap_->mapToWorld(map_width, map_height, world_x_max, world_y_max);

        RCLCPP_INFO(node_->get_logger(), 
                   "地图边界: X[%.2f, %.2f], Y[%.2f, %.2f], 分辨率: %.3f",
                   world_x_min, world_x_max, world_y_min, world_y_max, map_resolution);

        // 3.确定起始角落和目标角落
        geometry_msgs::msg::PoseStamped start_corner, end_corner;
        start_corner.header = global_path.header;
        end_corner.header = global_path.header;
        
        if (start_corner_ == "bottom_left") {
            start_corner.pose.position.x = world_x_min + 1.0;  // 留出1米边距
            start_corner.pose.position.y = world_y_min + 1.0;
            end_corner.pose.position.x = world_x_max - 1.0;
            end_corner.pose.position.y = world_y_max - 1.0;
        } else if (start_corner_ == "bottom_right") {
            start_corner.pose.position.x = world_x_max - 1.0;
            start_corner.pose.position.y = world_y_min + 1.0;
            end_corner.pose.position.x = world_x_min + 1.0;
            end_corner.pose.position.y = world_y_max - 1.0;
        } else if (start_corner_ == "top_left") {
            start_corner.pose.position.x = world_x_min + 1.0;
            start_corner.pose.position.y = world_y_max - 1.0;
            end_corner.pose.position.x = world_x_max - 1.0;
            end_corner.pose.position.y = world_y_min + 1.0;
        } else {  // top_right
            start_corner.pose.position.x = world_x_max - 1.0;
            start_corner.pose.position.y = world_y_max - 1.0;
            end_corner.pose.position.x = world_x_min + 1.0;
            end_corner.pose.position.y = world_y_min + 1.0;
        }

        // 4.生成蛇形路径点
        std::vector<geometry_msgs::msg::PoseStamped> path_points;
        
        // 首先从当前位置移动到起始角落
        path_points.push_back(start);
        path_points.push_back(start_corner);
        
        // 生成蛇形覆盖路径
        generateSnakePath(start_corner, end_corner, path_points);
        
        // 最后移动到目标位置
        path_points.push_back(end_corner);
        path_points.push_back(goal);

        // 5.对路径进行插值和平滑处理
        for (size_t i = 0; i < path_points.size(); ++i) {
            path_points[i].header = global_path.header;
            
            // 添加当前点到路径
            global_path.poses.push_back(path_points[i]);
            
            // 如果不是最后一个点，进行插值
            if (i < path_points.size() - 1) {
                interpolatePathSegment(path_points[i], path_points[i + 1], global_path);
            }
        }

        RCLCPP_INFO(node_->get_logger(), "生成了包含 %zu 个点的蛇形路径", global_path.poses.size());
        return global_path;
    }

    void CustomPlanner::generateSnakePath(
        const geometry_msgs::msg::PoseStamped &start_corner,
        const geometry_msgs::msg::PoseStamped &end_corner,
        std::vector<geometry_msgs::msg::PoseStamped> &path_points)
    {
        double start_x = start_corner.pose.position.x;
        double start_y = start_corner.pose.position.y;
        double end_x = end_corner.pose.position.x;
        double end_y = end_corner.pose.position.y;
        
        // 计算覆盖区域
        double coverage_width = std::abs(end_x - start_x);
        double coverage_height = std::abs(end_y - start_y);
        
        // 计算蛇形路径的行数和列数
        int rows = static_cast<int>(coverage_height / coverage_resolution_) + 1;
        int cols = static_cast<int>(coverage_width / coverage_resolution_) + 1;
        
        RCLCPP_INFO(node_->get_logger(), 
                   "蛇形路径: %d行 x %d列, 分辨率: %.2f", rows, cols, coverage_resolution_);
        
        // 生成蛇形路径点
        for (int row = 0; row < rows; ++row) {
            if (row % 2 == 0) {
                // 从左到右
                for (int col = 0; col < cols; ++col) {
                    geometry_msgs::msg::PoseStamped point;
                    point.header = start_corner.header;
                    point.pose.position.x = start_x + col * coverage_resolution_;
                    point.pose.position.y = start_y + row * coverage_resolution_;
                    point.pose.position.z = 0.0;
                    
                    // 设置朝向（朝向移动方向）
                    if (col < cols - 1) {
                        point.pose.orientation.w = 1.0;
                        point.pose.orientation.x = 0.0;
                        point.pose.orientation.y = 0.0;
                        point.pose.orientation.z = 0.0;
                    }
                    
                    path_points.push_back(point);
                }
            } else {
                // 从右到左
                for (int col = cols - 1; col >= 0; --col) {
                    geometry_msgs::msg::PoseStamped point;
                    point.header = start_corner.header;
                    point.pose.position.x = start_x + col * coverage_resolution_;
                    point.pose.position.y = start_y + row * coverage_resolution_;
                    point.pose.position.z = 0.0;
                    
                    // 设置朝向（朝向移动方向）
                    if (col > 0) {
                        point.pose.orientation.w = 1.0;
                        point.pose.orientation.x = 0.0;
                        point.pose.orientation.y = 0.0;
                        point.pose.orientation.z = 0.0;
                    }
                    
                    path_points.push_back(point);
                }
            }
        }
    }

    void CustomPlanner::interpolatePathSegment(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &end,
        nav_msgs::msg::Path &global_path)
    {
        double dx = end.pose.position.x - start.pose.position.x;
        double dy = end.pose.position.y - start.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance <= interpolation_resolution_) {
            return;  // 距离太短，不需要插值
        }
        
        int num_points = static_cast<int>(distance / interpolation_resolution_);
        
        for (int i = 1; i < num_points; ++i) {
            double ratio = static_cast<double>(i) / num_points;
            
            geometry_msgs::msg::PoseStamped interpolated_point;
            interpolated_point.header = global_path.header;
            interpolated_point.pose.position.x = start.pose.position.x + ratio * dx;
            interpolated_point.pose.position.y = start.pose.position.y + ratio * dy;
            interpolated_point.pose.position.z = 0.0;
            
            // 简单的线性插值朝向
            interpolated_point.pose.orientation = start.pose.orientation;
            
            global_path.poses.push_back(interpolated_point);
        }
    }

} // namespace nav2_custom_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,
                       nav2_core::GlobalPlanner)