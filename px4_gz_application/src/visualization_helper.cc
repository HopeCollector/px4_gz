#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace px4_gz {
class visualization_helper : public rclcpp::Node {
public:
  explicit visualization_helper(const rclcpp::NodeOptions &options)
      : Node("visualization_helper", options) {
    pub_drone_ =
        create_publisher<visualization_msgs::msg::Marker>("pub/drone", 5);
    pub_world_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("pub/world", 5);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_pub_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    timer_mesh_ = create_wall_timer(100ms, [this]() { this->pub_mesh_cb(); });
    timer_tf_static_ = create_wall_timer(1s, [this]() { this->tf_static_cb(); });

    declare_parameter("world_file_path", "");
    get_parameter("world_file_path", world_file_path_);
  }

private:
  void pub_mesh_cb() {
    try
    {
      auto t =
          tf_buffer_->lookupTransform("odom", "x500_lidar", tf2::TimePointZero);
    }
    catch(const std::exception& e)
    {}

    visualization_msgs::msg::Marker drone_msg;
    drone_msg.header.frame_id = "x500_lidar/base_link";
    drone_msg.header.stamp = get_clock()->now();
    drone_msg.action = visualization_msgs::msg::Marker::ADD;
    drone_msg.ns = "px4_gz";
    drone_msg.id = 0;
    drone_msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    drone_msg.mesh_resource =
        "https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/"
        "simulation/gz/models/x500/meshes/NXP-HGD-CF.dae";
    drone_msg.scale.x = 1.0;
    drone_msg.scale.y = 1.0;
    drone_msg.scale.z = 1.0;
    drone_msg.pose.position.x = 0;
    drone_msg.pose.position.y = 0;
    drone_msg.pose.position.z = 0;
    drone_msg.pose.orientation.w = 1.0;
    drone_msg.pose.orientation.x = 0.0;
    drone_msg.pose.orientation.y = 0.0;
    drone_msg.pose.orientation.z = 0.0;
    pub_drone_->publish(drone_msg);
    visualization_msgs::msg::MarkerArray world_msg;
  }

  void tf_static_cb() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "x500_lidar/link";
    msg.child_frame_id = "x500_lidar/link/gpu_lidar";
    tf_static_pub_->sendTransform(msg);
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_world_;
  rclcpp::TimerBase::SharedPtr timer_mesh_;
  rclcpp::TimerBase::SharedPtr timer_tf_static_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_pub_;
  std::string world_file_path_;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::visualization_helper)