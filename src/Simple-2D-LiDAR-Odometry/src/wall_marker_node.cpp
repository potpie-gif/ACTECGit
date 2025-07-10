#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>
#include <algorithm>
#include <vector>
#include <mosquitto.h>
#include <thread>
#include <atomic>
#include <string>

class WallMarkerNode : public rclcpp::Node
{
public:
    WallMarkerNode() : Node("wall_marker_node"), marker_id_(0), mosq_(nullptr), running_(true)
    {
        scan_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "ldlidar_node/scan", 10, std::bind(&WallMarkerNode::scanCallback, this, std::placeholders::_1));
        
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/wall_marker", 10);
        
        // Initialize MQTT
        initializeMQTT();
        
        RCLCPP_INFO(this->get_logger(), "Wall Marker Node started with MQTT client");
    }
    
    ~WallMarkerNode()
    {
        running_ = false;
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

private:
    void initializeMQTT()
    {
        mosquitto_lib_init();
        
        mosq_ = mosquitto_new("wall_marker_client", true, this);
        if (!mosq_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create MQTT client");
            return;
        }
        
        mosquitto_connect_callback_set(mosq_, [](struct mosquitto *mosq, void *obj, int result) {
            auto* node = static_cast<WallMarkerNode*>(obj);
            if (result == 0) {
                RCLCPP_INFO(node->get_logger(), "Connected to MQTT broker");
                mosquitto_subscribe(mosq, nullptr, "topic/qr", 0);
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to connect to MQTT broker");
            }
        });
        
        mosquitto_message_callback_set(mosq_, [](struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
            auto* node = static_cast<WallMarkerNode*>(obj);
            if (msg->payloadlen > 0) {
                std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
                RCLCPP_INFO(node->get_logger(), "MQTT message received from topic '%s': %s", msg->topic, payload.c_str());
                node->handleMQTTMessage(payload);
            }
        });
        
        mosquitto_username_pw_set(mosq_, "admin", "password");
        
        int rc = mosquitto_connect(mosq_, "192.168.0.138", 1883, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect to MQTT broker: %s", mosquitto_strerror(rc));
            return;
        }
        
        mosquitto_loop_start(mosq_);
    }
    
    void handleMQTTMessage(const std::string& message)
    {
        RCLCPP_INFO(this->get_logger(), "MQTT message received: %s", message.c_str());
        
        if (latest_scan_) {
            findAndMarkNearestWall();
        } else {
            RCLCPP_WARN(this->get_logger(), "No laser scan data available");
        }
    }
    
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        latest_scan_ = msg;
    }
    
    void findAndMarkNearestWall()
    {
        if (!latest_scan_) return;
        
        // Find the nearest wall in the negative y direction (opposite side of y-axis)
        // This corresponds to angles around -90 degrees (-π/2 radians)
        double target_angle = -M_PI_2; // -90 degrees
        double angle_tolerance = M_PI / 6; // 30 degrees tolerance to capture more wall area
        
        double nearest_distance = std::numeric_limits<double>::max();
        double nearest_angle = 0.0;
        bool found_wall = false;
        
        for (size_t i = 0; i < latest_scan_->ranges.size(); ++i) {
            double range = latest_scan_->ranges[i];
            
            // Skip invalid readings and robot parts (closer than 0.09m)
            if (std::isnan(range) || std::isinf(range) || range < latest_scan_->range_min || range > latest_scan_->range_max || range < 0.09) {
                continue;
            }
            
            double angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
            
            // Normalize angle to [-π, π]
            while (angle > M_PI) angle -= 2.0 * M_PI;
            while (angle < -M_PI) angle += 2.0 * M_PI;
            
            // Check if angle is in the negative y direction (opposite side of y-axis)
            double angle_diff = std::abs(angle - target_angle);
            if (angle_diff > M_PI) angle_diff = 2.0 * M_PI - angle_diff;
            
            if (angle_diff <= angle_tolerance) {
                // Check if this is the nearest wall point
                if (range < nearest_distance) {
                    nearest_distance = range;
                    nearest_angle = angle;
                    found_wall = true;
                }
            }
        }
        
        if (found_wall) {
            // Publish marker for the nearest wall point
            publishWallMarker(nearest_angle, nearest_distance);
            
            RCLCPP_INFO(this->get_logger(), "Found nearest wall at distance %.3f meters in the negative y direction", nearest_distance);
        } else {
            // No wall found, mark a point 0.5m in negative y direction
            publishWallMarker(-M_PI_2, 0.5);
            
            RCLCPP_WARN(this->get_logger(), "No wall found in the negative y direction, marking default point at 0.5m");
        }
    }
    
    void clearPreviousMarkers()
    {
        // Delete all previous markers
        for (int id : previous_marker_ids_) {
            auto marker = visualization_msgs::msg::Marker();
            marker.header.frame_id = latest_scan_->header.frame_id;
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "wall_markers";
            marker.id = id;
            marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_publisher_->publish(marker);
        }
        previous_marker_ids_.clear();
    }
    
    void publishWallMarker(double angle, double distance)
    {
        // Validate inputs
        if (std::isnan(angle) || std::isnan(distance) || std::isinf(angle) || std::isinf(distance)) {
            RCLCPP_ERROR(this->get_logger(), "Invalid angle (%f) or distance (%f) - contains NaN or Inf", angle, distance);
            return;
        }
        
        auto marker = visualization_msgs::msg::Marker();
        
        marker.header.frame_id = latest_scan_->header.frame_id;
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "wall_markers";
        marker.id = marker_id_;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Calculate position
        double x = distance * std::cos(angle);
        double y = distance * std::sin(angle);
        
        // Validate calculated position
        if (std::isnan(x) || std::isnan(y) || std::isinf(x) || std::isinf(y)) {
            RCLCPP_ERROR(this->get_logger(), "Calculated position is invalid - x: %f, y: %f (angle: %f, distance: %f)", x, y, angle, distance);
            return;
        }
        
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = 0.0;
        
        RCLCPP_INFO(this->get_logger(), "Publishing wall marker at position x: %f, y: %f (angle: %f, distance: %f)", x, y, angle, distance);
        
        // Set orientation
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        marker.pose.orientation = tf2::toMsg(q);
        
        // Set scale
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        
        // Set color (red)
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        
        // Set lifetime (0 means forever)
        marker.lifetime = rclcpp::Duration::from_seconds(0.0);
        
        // Track this marker ID for future deletion
        previous_marker_ids_.push_back(marker_id_);
        marker_id_++;
        
        marker_publisher_->publish(marker);
    }
    
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
    
    struct mosquitto *mosq_;
    std::atomic<bool> running_;
    
    sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
    int marker_id_;
    std::vector<int> previous_marker_ids_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallMarkerNode>());
    rclcpp::shutdown();
    return 0;
}
