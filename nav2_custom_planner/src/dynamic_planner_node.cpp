#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vector>
#include <memory>
#include <cmath>
#include <queue>
#include <unordered_set>

struct GridPoint {
    int x, y;
    double g_cost, h_cost, f_cost;
    GridPoint* parent;
    
    GridPoint(int x, int y) : x(x), y(y), g_cost(0), h_cost(0), f_cost(0), parent(nullptr) {}
    
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
    
    bool operator<(const GridPoint& other) const {
        return f_cost > other.f_cost;  // 优先队列需要大顶堆
    }
};

struct GridPointHash {
    size_t operator()(const GridPoint& point) const {
        return std::hash<int>()(point.x) ^ (std::hash<int>()(point.y) << 1);
    }
};

class DynamicPlannerNode : public rclcpp::Node
{
public:
    DynamicPlannerNode() : Node("dynamic_planner_node")
    {
        // 参数初始化
        this->declare_parameter("planning_frequency", 5.0);
        this->declare_parameter("robot_radius", 0.3);
        this->declare_parameter("goal_tolerance", 0.1);
        this->declare_parameter("max_linear_vel", 0.5);
        this->declare_parameter("max_angular_vel", 1.0);
        this->declare_parameter("obstacle_threshold", 0.5);
        
        planning_frequency_ = this->get_parameter("planning_frequency").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        max_linear_vel_ = this->get_parameter("max_linear_vel").as_double();
        max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_threshold").as_double();
        
        // 创建发布者和订阅者
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&DynamicPlannerNode::mapCallback, this, std::placeholders::_1));
            
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, std::bind(&DynamicPlannerNode::poseCallback, this, std::placeholders::_1));
            
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DynamicPlannerNode::laserCallback, this, std::placeholders::_1));
        
        // 服务
        start_planning_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_planning", std::bind(&DynamicPlannerNode::startPlanningCallback, this, 
                                       std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 定期进行路径规划
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / planning_frequency_)),
            std::bind(&DynamicPlannerNode::planningTimerCallback, this));
            
        // 初始化状态
        has_map_ = false;
        has_pose_ = false;
        planning_active_ = false;
        current_goal_index_ = 0;
        
        RCLCPP_INFO(this->get_logger(), "动态路径规划节点已启动");
        RCLCPP_INFO(this->get_logger(), "规划频率: %.1f Hz, 机器人半径: %.2f m", planning_frequency_, robot_radius_);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        current_map_ = *map;
        has_map_ = true;
        RCLCPP_DEBUG(this->get_logger(), "收到地图更新: %dx%d", map->info.width, map->info.height);
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        current_pose_ = *pose;
        has_pose_ = true;
    }
    
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 检查前方是否有障碍物
        bool obstacle_detected = false;
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            if (std::isnan(scan->ranges[i]) || std::isinf(scan->ranges[i])) {
                continue;
            }
            
            double range = scan->ranges[i];
            double angle = scan->angle_min + i * scan->angle_increment;
            
            // 检查前方±30度范围内的障碍物
            if (abs(angle) < M_PI / 6 && range < obstacle_threshold_) {
                obstacle_detected = true;
                break;
            }
        }
        
        if (obstacle_detected && planning_active_) {
            RCLCPP_WARN(this->get_logger(), "检测到障碍物，重新规划路径");
            replanPath();
        }
    }
    
    void startPlanningCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                              std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (!has_map_ || !has_pose_) {
            response->success = false;
            response->message = "地图或机器人位姿不可用";
            return;
        }
        
        planning_active_ = true;
        current_goal_index_ = 0;
        
        // 生成清淤任务目标点
        generateDredgingGoals();
        
        response->success = true;
        response->message = "路径规划已启动";
        RCLCPP_INFO(this->get_logger(), "清淤任务路径规划已启动");
    }
    
    void planningTimerCallback()
    {
        if (!planning_active_ || !has_map_ || !has_pose_) {
            return;
        }
        
        // 检查是否到达当前目标点
        if (current_goal_index_ < dredging_goals_.size()) {
            double distance = calculateDistance(current_pose_.pose.position, 
                                             dredging_goals_[current_goal_index_].pose.position);
            
            if (distance < goal_tolerance_) {
                RCLCPP_INFO(this->get_logger(), "到达目标点 %d", current_goal_index_);
                current_goal_index_++;
                
                if (current_goal_index_ >= dredging_goals_.size()) {
                    RCLCPP_INFO(this->get_logger(), "所有清淤目标已完成！");
                    planning_active_ = false;
                    stopRobot();
                    return;
                }
            }
        }
        
        // 规划到下一个目标点的路径
        if (current_goal_index_ < dredging_goals_.size()) {
            planPathToGoal(dredging_goals_[current_goal_index_]);
        }
    }
    
    void generateDredgingGoals()
    {
        dredging_goals_.clear();
        
        // 生成4x4网格的清淤目标点
        double grid_size = 5.0;
        double start_x = -10.0;
        double start_y = -10.0;
        
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                geometry_msgs::msg::PoseStamped goal;
                goal.header.frame_id = "map";
                goal.pose.position.x = start_x + i * grid_size + grid_size / 2.0;
                goal.pose.position.y = start_y + j * grid_size + grid_size / 2.0;
                goal.pose.position.z = 0.0;
                goal.pose.orientation.w = 1.0;
                
                dredging_goals_.push_back(goal);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "生成了 %zu 个清淤目标点", dredging_goals_.size());
    }
    
    void planPathToGoal(const geometry_msgs::msg::PoseStamped& goal)
    {
        if (!has_map_) {
            return;
        }
        
        // 将世界坐标转换为地图坐标
        int start_x, start_y, goal_x, goal_y;
        if (!worldToMap(current_pose_.pose.position.x, current_pose_.pose.position.y, start_x, start_y) ||
            !worldToMap(goal.pose.position.x, goal.pose.position.y, goal_x, goal_y)) {
            RCLCPP_WARN(this->get_logger(), "坐标转换失败");
            return;
        }
        
        // 使用A*算法规划路径
        std::vector<geometry_msgs::msg::PoseStamped> path_points;
        if (planAStarPath(start_x, start_y, goal_x, goal_y, path_points)) {
            // 发布规划路径
            nav_msgs::msg::Path path;
            path.header = current_pose_.header;
            path.header.frame_id = "map";
            path.poses = path_points;
            path_pub_->publish(path);
            
            // 执行路径跟踪
            executePath(path_points);
        } else {
            RCLCPP_WARN(this->get_logger(), "无法规划到目标点的路径");
        }
    }
    
    bool planAStarPath(int start_x, int start_y, int goal_x, int goal_y, 
                       std::vector<geometry_msgs::msg::PoseStamped>& path_points)
    {
        std::priority_queue<GridPoint> open_set;
        std::unordered_set<GridPoint, GridPointHash> closed_set;
        
        GridPoint start_point(start_x, start_y);
        start_point.g_cost = 0;
        start_point.h_cost = heuristic(start_x, start_y, goal_x, goal_y);
        start_point.f_cost = start_point.g_cost + start_point.h_cost;
        
        open_set.push(start_point);
        
        while (!open_set.empty()) {
            GridPoint current = open_set.top();
            open_set.pop();
            
            if (current.x == goal_x && current.y == goal_y) {
                // 重建路径
                reconstructPath(&current, path_points);
                return true;
            }
            
            closed_set.insert(current);
            
            // 检查8个方向的邻居
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;
                    
                    int neighbor_x = current.x + dx;
                    int neighbor_y = current.y + dy;
                    
                    if (!isValidGridPoint(neighbor_x, neighbor_y) || 
                        isObstacle(neighbor_x, neighbor_y) ||
                        closed_set.find(GridPoint(neighbor_x, neighbor_y)) != closed_set.end()) {
                        continue;
                    }
                    
                    double new_g_cost = current.g_cost + (dx != 0 && dy != 0 ? 1.414 : 1.0);
                    
                    GridPoint neighbor(neighbor_x, neighbor_y);
                    neighbor.g_cost = new_g_cost;
                    neighbor.h_cost = heuristic(neighbor_x, neighbor_y, goal_x, goal_y);
                    neighbor.f_cost = new_g_cost + neighbor.h_cost;
                    neighbor.parent = &current;
                    
                    open_set.push(neighbor);
                }
            }
        }
        
        return false;  // 没有找到路径
    }
    
    void reconstructPath(GridPoint* goal_point, std::vector<geometry_msgs::msg::PoseStamped>& path_points)
    {
        path_points.clear();
        std::vector<GridPoint> reverse_path;
        
        GridPoint* current = goal_point;
        while (current != nullptr) {
            reverse_path.push_back(*current);
            current = current->parent;
        }
        
        // 反转路径并转换为世界坐标
        for (int i = reverse_path.size() - 1; i >= 0; --i) {
            double world_x, world_y;
            mapToWorld(reverse_path[i].x, reverse_path[i].y, world_x, world_y);
            
            geometry_msgs::msg::PoseStamped pose;
            pose.header = current_pose_.header;
            pose.header.frame_id = "map";
            pose.pose.position.x = world_x;
            pose.pose.position.y = world_y;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.w = 1.0;
            
            path_points.push_back(pose);
        }
    }
    
    void executePath(const std::vector<geometry_msgs::msg::PoseStamped>& path_points)
    {
        if (path_points.empty()) {
            return;
        }
        
        // 找到路径中的下一个目标点
        size_t next_index = std::min(current_goal_index_ + 1, path_points.size() - 1);
        const auto& next_point = path_points[next_index];
        
        // 计算控制命令
        double distance = calculateDistance(current_pose_.pose.position, next_point.pose.position);
        double angle = calculateAngle(current_pose_.pose.position, next_point.pose.position);
        
        geometry_msgs::msg::Twist cmd_vel;
        
        // 简单的比例控制
        if (abs(angle) > 0.3) {
            // 角度太大，先转向
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = std::clamp(angle * 1.0, -max_angular_vel_, max_angular_vel_);
        } else {
            // 前进
            cmd_vel.linear.x = std::min(distance * 0.5, max_linear_vel_);
            cmd_vel.angular.z = std::clamp(angle * 0.5, -max_angular_vel_, max_angular_vel_);
        }
        
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    void replanPath()
    {
        if (current_goal_index_ < dredging_goals_.size()) {
            planPathToGoal(dredging_goals_[current_goal_index_]);
        }
    }
    
    void stopRobot()
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd_vel);
    }
    
    // 辅助函数
    double heuristic(int x1, int y1, int x2, int y2)
    {
        return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }
    
    bool isValidGridPoint(int x, int y)
    {
        return x >= 0 && x < static_cast<int>(current_map_.info.width) &&
               y >= 0 && y < static_cast<int>(current_map_.info.height);
    }
    
    bool isObstacle(int x, int y)
    {
        if (!isValidGridPoint(x, y)) {
            return true;
        }
        
        int index = y * current_map_.info.width + x;
        return current_map_.data[index] > 50;  // 代价值大于50认为是障碍物
    }
    
    bool worldToMap(double world_x, double world_y, int& map_x, int& map_y)
    {
        map_x = static_cast<int>((world_x - current_map_.info.origin.position.x) / current_map_.info.resolution);
        map_y = static_cast<int>((world_y - current_map_.info.origin.position.y) / current_map_.info.resolution);
        
        return isValidGridPoint(map_x, map_y);
    }
    
    void mapToWorld(int map_x, int map_y, double& world_x, double& world_y)
    {
        world_x = map_x * current_map_.info.resolution + current_map_.info.origin.position.x;
        world_y = map_y * current_map_.info.resolution + current_map_.info.origin.position.y;
    }
    
    double calculateDistance(const geometry_msgs::msg::Point& p1, const geometry_msgs::msg::Point& p2)
    {
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        return sqrt(dx * dx + dy * dy);
    }
    
    double calculateAngle(const geometry_msgs::msg::Point& from, const geometry_msgs::msg::Point& to)
    {
        double dx = to.x - from.x;
        double dy = to.y - from.y;
        return atan2(dy, dx);
    }
    
    // 参数
    double planning_frequency_;
    double robot_radius_;
    double goal_tolerance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double obstacle_threshold_;
    
    // 状态
    bool has_map_;
    bool has_pose_;
    bool planning_active_;
    size_t current_goal_index_;
    
    // 数据
    nav_msgs::msg::OccupancyGrid current_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    std::vector<geometry_msgs::msg::PoseStamped> dredging_goals_;
    
    // 发布者和订阅者
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_planning_srv_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicPlannerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

