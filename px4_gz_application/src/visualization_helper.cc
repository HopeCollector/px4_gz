#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ylt/struct_yaml/yaml_reader.h>

namespace px4_gz {
struct mesh_t {
  std::string type;
  std::string name;
  std::string frame_id;
  std::string mesh_resource;
  std::vector<double> position;
  std::vector<double> orientation;
  std::vector<double> scale;
};
REFLECTION(mesh_t, type, name, frame_id, mesh_resource, position, orientation,
           scale);

std::ostream &operator<<(std::ostream &os, mesh_t m) {
  os << m.name;
  return os;
}

class visualization_helper : public rclcpp::Node {
public:
  explicit visualization_helper(const rclcpp::NodeOptions &options)
      : Node("visualization_helper", options) {
    pub_drone_ =
        create_publisher<visualization_msgs::msg::Marker>("pub/drone", 5);
    pub_world_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("pub/world", 5);
    tf_static_pub_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    timer_50hz_ = create_wall_timer(20ms, [this]() { this->callback_50hz(); });
    timer_1hz_ = create_wall_timer(1s, [this]() { this->callback_1hz(); });

    declare_parameter("world_file_path", "");
    get_parameter("world_file_path", world_file_path_);
    RCLCPP_INFO_STREAM(get_logger(), world_file_path_);
  }

private:
  void callback_50hz() { pub_drone_msg(); }

  void callback_1hz() {
    pub_tf_static();
    pub_world_msg();
  }

  void pub_tf_static() {
    static bool is_created_msg = false;
    static geometry_msgs::msg::TransformStamped msg;
    if (!is_created_msg) {
      msg.header.stamp = get_clock()->now();
      msg.header.frame_id = "x500_lidar/link";
      msg.child_frame_id = "x500_lidar/link/gpu_lidar";
      is_created_msg = true;
    }
    tf_static_pub_->sendTransform(msg);
  }

  void pub_drone_msg() {
    static bool is_created_msg = false;
    static visualization_msgs::msg::Marker drone_msg;
    if (!is_created_msg) {
      drone_msg = create_mesh_msg(
          "https://raw.githubusercontent.com/PX4/PX4-Autopilot/main/Tools/"
          "simulation/gz/models/x500/meshes/NXP-HGD-CF.dae",
          {0, 0, 0, 1, 0, 0, 0});
      drone_msg.header.frame_id = "x500_lidar/base_link";
      drone_msg.id = 0;
      is_created_msg = true;
    }
    drone_msg.header.stamp = get_clock()->now();
    pub_drone_->publish(drone_msg);
  }

  void pub_world_msg() {
    static bool is_created_msg = false;
    static visualization_msgs::msg::MarkerArray world_msg;
    if (!is_created_msg) {
      int id = 1;
      std::ifstream ifs(world_file_path_);
      std::string content((std::istreambuf_iterator<char>(ifs)),
                          (std::istreambuf_iterator<char>()));
      std::vector<mesh_t> meshes;
      iguana::from_yaml(meshes, content);
      world_msg.markers.reserve(meshes.size());
      for (const auto &mesh : meshes) {
        auto msg = create_mesh_msg(mesh.mesh_resource,
                                   {mesh.position[0], mesh.position[1],
                                    mesh.position[2], mesh.orientation[0],
                                    mesh.orientation[1], mesh.orientation[2],
                                    mesh.orientation[3]});
        msg.header.frame_id = "odom";
        msg.header.stamp = get_clock()->now();
        msg.id = id++;
        world_msg.markers.push_back(msg);
      }
      is_created_msg = true;
    }
    pub_world_->publish(world_msg);
  }

  visualization_msgs::msg::Marker create_mesh_msg(std::string resource,
                                                  std::vector<double> pose) {
    visualization_msgs::msg::Marker msg;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.ns = "px4_gz";
    msg.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    msg.mesh_resource = resource;
    msg.scale.x = 1.0;
    msg.scale.y = 1.0;
    msg.scale.z = 1.0;
    msg.pose.position.x = pose[0];
    msg.pose.position.y = pose[1];
    msg.pose.position.z = pose[2];
    msg.pose.orientation.w = pose[3];
    msg.pose.orientation.x = pose[4];
    msg.pose.orientation.y = pose[5];
    msg.pose.orientation.z = pose[6];
    return msg;
  }

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_world_;
  rclcpp::TimerBase::SharedPtr timer_50hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_pub_;
  std::string world_file_path_;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::visualization_helper)