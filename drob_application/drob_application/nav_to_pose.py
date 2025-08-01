from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy

def main():
    rclpy.init()
    nav = BasicNavigator()
    nav.waitUntilNav2Active()
    go_pose = PoseStamped()
    go_pose.header.frame_id = 'map'
    go_pose.header.stamp = nav.get_clock().now().to_msg()
    go_pose.pose.position.x = 2.0 #x
    go_pose.pose.position.y = 1.0 #y
    go_pose.pose.orientation.w = 1.0
    nav.goToPose(go_pose)
    while not nav.isTaskComplete():
        feedback=nav.getFeedback()
        nav.get_logger().info(f'导航反馈{feedback.distance_remaining}')
    result = nav.getResult()
    nav.get_logger().info(f'导航任务完成，状态码{result}')

