#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include <vector>
#include <memory>
#include <string>
#include <chrono>

class DredgingTaskNode : public rclcpp::Node
{
public:
    DredgingTaskNode() : Node("dredging_task_node")
    {
        // 参数初始化
        this->declare_parameter("task_timeout", 300.0);  // 任务超时时间（秒）
        this->declare_parameter("area_coverage_threshold", 0.8);  // 区域覆盖阈值
        this->declare_parameter("dredging_duration", 10.0);  // 每个区域清淤持续时间（秒）
        
        task_timeout_ = this->get_parameter("task_timeout").as_double();
        area_coverage_threshold_ = this->get_parameter("area_coverage_threshold").as_double();
        dredging_duration_ = this->get_parameter("dredging_duration").as_double();
        
        // 创建发布者和订阅者
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/dredging_status", 10);
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose", 10, std::bind(&DredgingTaskNode::poseCallback, this, std::placeholders::_1));
            
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map", 10, std::bind(&DredgingTaskNode::mapCallback, this, std::placeholders::_1));
        
        // 服务
        start_task_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/start_dredging", std::bind(&DredgingTaskNode::startTaskCallback, this, 
                                       std::placeholders::_1, std::placeholders::_2));
                                       
        stop_task_srv_ = this->create_service<std_srvs::srv::Trigger>(
            "/stop_dredging", std::bind(&DredgingTaskNode::stopTaskCallback, this, 
                                      std::placeholders::_1, std::placeholders::_2));
        
        // 定时器 - 定期检查任务状态
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),  // 1Hz
            std::bind(&DredgingTaskNode::taskTimerCallback, this));
            
        // 初始化状态
        task_active_ = false;
        current_area_index_ = 0;
        area_start_time_ = this->now();
        task_start_time_ = this->now();
        
        // 初始化清淤区域
        initializeDredgingAreas();
        
        RCLCPP_INFO(this->get_logger(), "清淤任务管理节点已启动");
        RCLCPP_INFO(this->get_logger(), "任务超时: %.1f秒, 清淤持续时间: %.1f秒", task_timeout_, dredging_duration_);
    }

private:
    void initializeDredgingAreas()
    {
        dredging_areas_.clear();
        
        // 定义4x4网格的清淤区域
        double grid_size = 5.0;
        double start_x = -10.0;
        double start_y = -10.0;
        
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                DredgingArea area;
                area.center_x = start_x + i * grid_size + grid_size / 2.0;
                area.center_y = start_y + j * grid_size + grid_size / 2.0;
                area.size = grid_size;
                area.completed = false;
                area.coverage = 0.0;
                area.visited = false;
                
                dredging_areas_.push_back(area);
            }
        }
        
        RCLCPP_INFO(this->get_logger(), "初始化了 %zu 个清淤区域", dredging_areas_.size());
    }
    
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose)
    {
        current_pose_ = *pose;
        has_pose_ = true;
        
        if (task_active_) {
            updateAreaStatus();
        }
    }
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map)
    {
        current_map_ = *map;
        has_map_ = true;
        
        if (task_active_) {
            updateCoverageStatus();
        }
    }
    
    void startTaskCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                          std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (!has_pose_ || !has_map_) {
            response->success = false;
            response->message = "机器人位姿或地图不可用";
            return;
        }
        
        if (task_active_) {
            response->success = false;
            response->message = "清淤任务已在运行中";
            return;
        }
        
        // 启动任务
        task_active_ = true;
        current_area_index_ = 0;
        task_start_time_ = this->now();
        area_start_time_ = this->now();
        
        // 重置所有区域状态
        for (auto& area : dredging_areas_) {
            area.completed = false;
            area.coverage = 0.0;
            area.visited = false;
        }
        
        response->success = true;
        response->message = "清淤任务已启动";
        
        RCLCPP_INFO(this->get_logger(), "清淤任务已启动");
        publishStatus("任务已启动，开始清淤区域 0");
    }
    
    void stopTaskCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                         std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        if (!task_active_) {
            response->success = false;
            response->message = "没有正在运行的清淤任务";
            return;
        }
        
        // 停止任务
        task_active_ = false;
        
        response->success = true;
        response->message = "清淤任务已停止";
        
        RCLCPP_INFO(this->get_logger(), "清淤任务已停止");
        publishStatus("任务已停止");
    }
    
    void taskTimerCallback()
    {
        if (!task_active_) {
            return;
        }
        
        // 检查任务超时
        auto now = this->now();
        auto task_duration = (now - task_start_time_).seconds();
        
        if (task_duration > task_timeout_) {
            RCLCPP_WARN(this->get_logger(), "清淤任务超时，自动停止");
            task_active_ = false;
            publishStatus("任务超时，自动停止");
            return;
        }
        
        // 检查当前区域是否完成
        if (current_area_index_ < dredging_areas_.size()) {
            auto area_duration = (now - area_start_time_).seconds();
            
            if (area_duration >= dredging_duration_) {
                completeCurrentArea();
            }
        }
        
        // 检查是否所有区域都已完成
        if (current_area_index_ >= dredging_areas_.size()) {
            completeAllAreas();
        }
    }
    
    void updateAreaStatus()
    {
        if (!task_active_ || current_area_index_ >= dredging_areas_.size()) {
            return;
        }
        
        auto& current_area = dredging_areas_[current_area_index_];
        
        // 检查机器人是否在当前清淤区域内
        double distance = calculateDistance(current_pose_.pose.position.x, current_pose_.pose.position.y,
                                         current_area.center_x, current_area.center_y);
        
        if (distance <= current_area.size / 2.0) {
            if (!current_area.visited) {
                current_area.visited = true;
                area_start_time_ = this->now();
                RCLCPP_INFO(this->get_logger(), "进入清淤区域 %d", current_area_index_);
                publishStatus("进入清淤区域 " + std::to_string(current_area_index_));
            }
        }
    }
    
    void updateCoverageStatus()
    {
        if (!task_active_ || current_area_index_ >= dredging_areas_.size()) {
            return;
        }
        
        auto& current_area = dredging_areas_[current_area_index_];
        
        // 计算当前区域的覆盖率（简化计算）
        // 这里可以根据实际需求实现更复杂的覆盖率计算
        if (current_area.visited) {
            auto now = this->now();
            auto duration = (now - area_start_time_).seconds();
            current_area.coverage = std::min(duration / dredging_duration_, 1.0);
        }
    }
    
    void completeCurrentArea()
    {
        if (current_area_index_ >= dredging_areas_.size()) {
            return;
        }
        
        auto& current_area = dredging_areas_[current_area_index_];
        current_area.completed = true;
        
        RCLCPP_INFO(this->get_logger(), "清淤区域 %d 完成，覆盖率: %.1f%%", 
                   current_area_index_, current_area.coverage * 100.0);
        
        publishStatus("区域 " + std::to_string(current_area_index_) + " 完成，覆盖率: " + 
                     std::to_string(static_cast<int>(current_area.coverage * 100)) + "%");
        
        // 移动到下一个区域
        current_area_index_++;
        
        if (current_area_index_ < dredging_areas_.size()) {
            area_start_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "开始清淤区域 %d", current_area_index_);
            publishStatus("开始清淤区域 " + std::to_string(current_area_index_));
        }
    }
    
    void completeAllAreas()
    {
        if (!task_active_) {
            return;
        }
        
        task_active_ = false;
        
        // 计算总体完成情况
        int completed_areas = 0;
        double total_coverage = 0.0;
        
        for (const auto& area : dredging_areas_) {
            if (area.completed) {
                completed_areas++;
                total_coverage += area.coverage;
            }
        }
        
        double average_coverage = completed_areas > 0 ? total_coverage / completed_areas : 0.0;
        
        RCLCPP_INFO(this->get_logger(), "所有清淤区域完成！完成区域: %d/%zu, 平均覆盖率: %.1f%%", 
                   completed_areas, dredging_areas_.size(), average_coverage * 100.0);
        
        publishStatus("任务完成！完成区域: " + std::to_string(completed_areas) + "/" + 
                     std::to_string(dredging_areas_.size()) + ", 平均覆盖率: " + 
                     std::to_string(static_cast<int>(average_coverage * 100)) + "%");
    }
    
    void publishStatus(const std::string& status)
    {
        std_msgs::msg::String status_msg;
        status_msg.data = status;
        status_pub_->publish(status_msg);
    }
    
    double calculateDistance(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return sqrt(dx * dx + dy * dy);
    }
    
    // 清淤区域结构
    struct DredgingArea {
        double center_x, center_y;
        double size;
        bool completed;
        double coverage;
        bool visited;
    };
    
    // 参数
    double task_timeout_;
    double area_coverage_threshold_;
    double dredging_duration_;
    
    // 状态
    bool task_active_;
    size_t current_area_index_;
    bool has_pose_;
    bool has_map_;
    
    // 数据
    geometry_msgs::msg::PoseStamped current_pose_;
    nav_msgs::msg::OccupancyGrid current_map_;
    std::vector<DredgingArea> dredging_areas_;
    
    // 时间
    rclcpp::Time task_start_time_;
    rclcpp::Time area_start_time_;
    
    // 发布者和订阅者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    // 服务
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_task_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_task_srv_;
    
    // 定时器
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DredgingTaskNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

