#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class TfPosePublisher : public rclcpp::Node {
public:
  TfPosePublisher() : Node("tf_pose_publisher") {
    this->declare_parameter<std::string>("global_frame", "map");
    this->declare_parameter<std::string>("base_frame", "base_footprint");
    this->declare_parameter<double>("publish_rate", 10.0);

    global_frame_ = this->get_parameter("global_frame").as_string();
    base_frame_ = this->get_parameter("base_frame").as_string();
    publish_rate_ = this->get_parameter("publish_rate").as_double();

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/robot_pose", 10);

    auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_));
    timer_ = this->create_wall_timer(period, std::bind(&TfPosePublisher::onTimer, this));

    RCLCPP_INFO(this->get_logger(), "tf_pose_publisher started. global_frame=%s base_frame=%s rate=%.1f", 
                global_frame_.c_str(), base_frame_.c_str(), publish_rate_);
  }

private:
  void onTimer() {
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF lookup failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = tf.header;
    pose.header.frame_id = global_frame_;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;

    pose_pub_->publish(pose);
  }

  std::string global_frame_;
  std::string base_frame_;
  double publish_rate_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TfPosePublisher>());
  rclcpp::shutdown();
  return 0;
}
