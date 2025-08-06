#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

class FourWheelController : public rclcpp::Node
{
public:
    FourWheelController() : Node("four_wheel_controller")
    {
        // 创建发布者 - 使用Float64MultiArray
        left_front_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drob_left_front_velocity_controller/commands", 10);
        right_front_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drob_right_front_velocity_controller/commands", 10);
        left_back_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drob_left_back_velocity_controller/commands", 10);
        right_back_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/drob_right_back_velocity_controller/commands", 10);
        
        // 创建odom发布者
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        
        // 创建订阅者
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&FourWheelController::cmdVelCallback, this, std::placeholders::_1));
        
        // 订阅joint_states用于里程计计算
        joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&FourWheelController::jointStatesCallback, this, std::placeholders::_1));
        
        // 创建tf广播器
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        
        // 声明参数
        this->declare_parameter("wheel_separation", 0.3);  // 轮距：左右轮之间的距离
        this->declare_parameter("wheel_radius", 0.05);      // 轮半径
        this->declare_parameter("max_linear_velocity", 2.0);
        this->declare_parameter("max_angular_velocity", 3.0);
        this->declare_parameter("odom_frame", "odom");
        this->declare_parameter("base_frame", "base_footprint");
        
        // 获取参数
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        max_linear_velocity_ = this->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = this->get_parameter("max_angular_velocity").as_double();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        
        // 初始化里程计变量
        x_ = 0.0;
        y_ = 0.0;
        theta_ = 0.0;
        last_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "四轮控制器已启动");
        RCLCPP_INFO(this->get_logger(), "轮距: %.3f m, 轮半径: %.3f m", wheel_separation_, wheel_radius_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 限制速度
        double linear_x = std::clamp(msg->linear.x, -max_linear_velocity_, max_linear_velocity_);
        double angular_z = std::clamp(msg->angular.z, -max_angular_velocity_, max_angular_velocity_);
        
        // 使用差速转向运动学
        // 对于四轮机器人，左右轮速度差产生转向
        double left_velocity, right_velocity;
        
        // 差速转向公式：
        // v_left = v_linear - (v_angular * wheel_separation) / 2
        // v_right = v_linear + (v_angular * wheel_separation) / 2
        left_velocity = linear_x - (angular_z * wheel_separation_) / 2.0;
        right_velocity = linear_x + (angular_z * wheel_separation_) / 2.0;
        
        // 转换为轮子角速度 (rad/s)
        double left_angular_velocity = left_velocity / wheel_radius_;
        double right_angular_velocity = right_velocity / wheel_radius_;
        
        // 发布轮子速度命令 - 使用Float64MultiArray
        auto left_front_msg = std_msgs::msg::Float64MultiArray();
        auto right_front_msg = std_msgs::msg::Float64MultiArray();
        auto left_back_msg = std_msgs::msg::Float64MultiArray();
        auto right_back_msg = std_msgs::msg::Float64MultiArray();
        
        // 设置数据
        left_front_msg.data = {left_angular_velocity};
        right_front_msg.data = {right_angular_velocity};
        left_back_msg.data = {left_angular_velocity};
        right_back_msg.data = {right_angular_velocity};
        
        left_front_pub_->publish(left_front_msg);
        right_front_pub_->publish(right_front_msg);
        left_back_pub_->publish(left_back_msg);
        right_back_pub_->publish(right_back_msg);
        
        // 调试信息
        RCLCPP_INFO(this->get_logger(), 
                     "cmd_vel: (%.2f, %.2f) -> 轮速: 左(%.2f), 右(%.2f)", 
                     linear_x, angular_z, left_angular_velocity, right_angular_velocity);
    }
    
    void jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 查找轮子关节
        int lfj_idx = -1, rfj_idx = -1, lbj_idx = -1, rbj_idx = -1;
        
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "lfj") lfj_idx = i;
            else if (msg->name[i] == "rfj") rfj_idx = i;
            else if (msg->name[i] == "lbj") lbj_idx = i;
            else if (msg->name[i] == "rbj") rbj_idx = i;
        }
        
        if (lfj_idx >= 0 && rfj_idx >= 0 && lbj_idx >= 0 && rbj_idx >= 0) {
            // 计算平均轮子速度
            double left_velocity = (msg->velocity[lfj_idx] + msg->velocity[lbj_idx]) / 2.0;
            double right_velocity = (msg->velocity[rfj_idx] + msg->velocity[rbj_idx]) / 2.0;
            
            // 计算里程计
            updateOdometry(left_velocity, right_velocity);
        }
    }
    
    void updateOdometry(double left_velocity, double right_velocity)
    {
        auto current_time = this->now();
        double dt = (current_time - last_time_).seconds();
        
        if (dt > 0.0) {
            // 计算线速度和角速度
            double linear_velocity = (left_velocity + right_velocity) * wheel_radius_ / 2.0;
            double angular_velocity = (right_velocity - left_velocity) * wheel_radius_ / wheel_separation_;
            
            // 更新位置
            double delta_x = linear_velocity * cos(theta_) * dt;
            double delta_y = linear_velocity * sin(theta_) * dt;
            double delta_theta = angular_velocity * dt;
            
            x_ += delta_x;
            y_ += delta_y;
            theta_ += delta_theta;
            
            // 发布里程计消息
            auto odom_msg = nav_msgs::msg::Odometry();
            odom_msg.header.stamp = current_time;
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = base_frame_;
            
            odom_msg.pose.pose.position.x = x_;
            odom_msg.pose.pose.position.y = y_;
            odom_msg.pose.pose.position.z = 0.0;
            
            // 转换为四元数
            tf2::Quaternion q;
            q.setRPY(0, 0, theta_);
            odom_msg.pose.pose.orientation.x = q.x();
            odom_msg.pose.pose.orientation.y = q.y();
            odom_msg.pose.pose.orientation.z = q.z();
            odom_msg.pose.pose.orientation.w = q.w();
            
            odom_msg.twist.twist.linear.x = linear_velocity;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z = angular_velocity;
            
            odom_pub_->publish(odom_msg);
            
            // 发布tf变换
            auto transform = geometry_msgs::msg::TransformStamped();
            transform.header.stamp = current_time;
            transform.header.frame_id = odom_frame_;
            transform.child_frame_id = base_frame_;
            transform.transform.translation.x = x_;
            transform.transform.translation.y = y_;
            transform.transform.translation.z = 0.0;
            transform.transform.rotation.x = q.x();
            transform.transform.rotation.y = q.y();
            transform.transform.rotation.z = q.z();
            transform.transform.rotation.w = q.w();
            
            tf_broadcaster_->sendTransform(transform);
        }
        
        last_time_ = current_time;
    }
    
    // 发布者 - 使用Float64MultiArray
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_front_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_front_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr left_back_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr right_back_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    
    // 订阅者
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    
    // TF广播器
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // 参数
    double wheel_separation_;
    double wheel_radius_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    std::string odom_frame_;
    std::string base_frame_;
    
    // 里程计变量
    double x_, y_, theta_;
    rclcpp::Time last_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FourWheelController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
} 