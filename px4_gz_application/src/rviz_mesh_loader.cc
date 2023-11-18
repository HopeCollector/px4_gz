#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace px4_gz {
class rivz_mesh_loader : public rclcpp::Node {
public:
  explicit rivz_mesh_loader(const rclcpp::NodeOptions &options)
      : Node("rviz_mesh_loader", options) {
    pub_drone_ =
        create_publisher<visualization_msgs::msg::Marker>("pub/drone", 5);
    pub_world_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("pub/world", 5);
    using namespace std::chrono_literals;
    timer_ = create_wall_timer(100ms, [this]() { this->pub_mesh_cb(); });
  }

private:
  void pub_mesh_cb() {
    RCLCPP_INFO_STREAM(get_logger(), "hi");
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_world_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::rivz_mesh_loader)