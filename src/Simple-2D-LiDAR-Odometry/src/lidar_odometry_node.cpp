#include <cstdio>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"  // ✅ Static transform
#include "lidar_odometry/lidar_odometry.hpp"

class LidarOdometryNode : public rclcpp::Node
{
public:
  LidarOdometryNode() : Node("lidar_odometry_node")
  {
    RCLCPP_INFO(this->get_logger(), "lidar_odometry_node");

    parameter_initialization();

    double max_correspondence_distance;
    double transformation_epsilon;
    double maximum_iterations;
    std::string scan_topic_name;
    std::string odom_topic_name;

    this->get_parameter("max_correspondence_distance", max_correspondence_distance);
    this->get_parameter("transformation_epsilon", transformation_epsilon);
    this->get_parameter("maximum_iterations", maximum_iterations);
    this->get_parameter("scan_topic_name", scan_topic_name);
    this->get_parameter("odom_topic_name", odom_topic_name);

    RCLCPP_INFO(this->get_logger(), "===== Configuration =====");
    RCLCPP_INFO(this->get_logger(), "max_correspondence_distance: %.4f", max_correspondence_distance);
    RCLCPP_INFO(this->get_logger(), "transformation_epsilon: %.4f", transformation_epsilon);
    RCLCPP_INFO(this->get_logger(), "maximum_iterations %.4f", maximum_iterations);
    RCLCPP_INFO(this->get_logger(), "scan_topic_name: %s", scan_topic_name.c_str());
    RCLCPP_INFO(this->get_logger(), "odom_topic_name: %s", odom_topic_name.c_str());

    lidar_odometry_ptr = std::make_shared<LidarOdometry>(
      max_correspondence_distance, transformation_epsilon, maximum_iterations);

    odom_publisher = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_name, 100);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);  // ✅ Create static broadcaster

    rclcpp::QoS qos(10);
    qos.best_effort();

    scan_subscriber = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_name, qos,
      std::bind(&LidarOdometryNode::scan_callback, this, std::placeholders::_1)
    );

    publish_static_transform();  // ✅ Send static transform at startup
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber;
  std::shared_ptr<LidarOdometry> lidar_odometry_ptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;  // ✅ Member for static transform

  void parameter_initialization()
  {
    this->declare_parameter<double>("max_correspondence_distance", 1.0);
    this->declare_parameter<double>("transformation_epsilon", 0.005);
    this->declare_parameter<double>("maximum_iterations", 30);
    this->declare_parameter<std::string>("scan_topic_name", "ldlidar_node/scan");
    this->declare_parameter<std::string>("odom_topic_name", "odom");
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    auto point_cloud_msg = laser2cloudmsg(scan_msg);
    auto pcl_point_cloud = cloudmsg2cloud(point_cloud_msg);

    RCLCPP_INFO(this->get_logger(), "Point cloud size: %zu", pcl_point_cloud.size());

    if (pcl_point_cloud.size() < 20) {
      RCLCPP_WARN(this->get_logger(), "Point cloud too small (%zu points), skipping this scan.", pcl_point_cloud.size());
      return;
    }

    auto scan_data = std::make_shared<ScanData>();
    scan_data->timestamp = scan_msg->header.stamp.sec + scan_msg->header.stamp.nanosec / 1e9;
    scan_data->point_cloud = pcl_point_cloud;

    lidar_odometry_ptr->process_scan_data(scan_data);
    publish_odometry();
  }

  void publish_odometry()
  {
    auto state = lidar_odometry_ptr->get_state();
    std::string fixed_id = "odom";
    std::string child_id = "base_link";

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = fixed_id;
    odom_msg.child_frame_id = child_id;
    odom_msg.header.stamp = this->get_clock()->now();

    odom_msg.pose.pose = Eigen::toMsg(state->pose);
    odom_msg.twist.twist = Eigen::toMsg(state->velocity);

    odom_publisher->publish(odom_msg);

    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = odom_msg.header.stamp;
    odom_tf.header.frame_id = fixed_id;
    odom_tf.child_frame_id = child_id;
    odom_tf.transform.translation.x = state->pose.translation().x();
    odom_tf.transform.translation.y = state->pose.translation().y();
    odom_tf.transform.translation.z = state->pose.translation().z();
    odom_tf.transform.rotation = Eigen::toMsg(Eigen::Quaterniond(state->pose.rotation()));

    tf_broadcaster_->sendTransform(odom_tf);
  }

  void publish_static_transform()
  {
    geometry_msgs::msg::TransformStamped static_transform;
    static_transform.header.stamp = this->get_clock()->now();
    static_transform.header.frame_id = "base_link";
    static_transform.child_frame_id = "ldlidar_base";
    static_transform.transform.translation.x = 0.0;
    static_transform.transform.translation.y = 0.0;
    static_transform.transform.translation.z = 0.0;
    static_transform.transform.rotation.x = 0.0;
    static_transform.transform.rotation.y = 0.0;
    static_transform.transform.rotation.z = 0.0;
    static_transform.transform.rotation.w = 1.0;

    static_broadcaster_->sendTransform(static_transform);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarOdometryNode>());
  rclcpp::shutdown();
  return 0;
}
