#include <chrono>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <ylt/struct_yaml/yaml_reader.h>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

constexpr double ARROW_MAX_LEN = 1.0;

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
    pub_velocity_ =
        create_publisher<visualization_msgs::msg::Marker>("pub/velocity", 5);
    pub_world_ =
        create_publisher<visualization_msgs::msg::MarkerArray>("pub/world", 5);
    tf_static_pub_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    sub_odom_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "sub/odom", 5, std::bind(&visualization_helper::cb_odom, this, _1));
    timer_50hz_ = create_wall_timer(20ms, std::bind(&visualization_helper::callback_50hz, this));
    timer_1hz_ = create_wall_timer(1s, std::bind(&visualization_helper::callback_1hz, this));

    world_file_path_ = declare_parameter("world_file_path", "");
    odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
    RCLCPP_INFO_STREAM(get_logger(), world_file_path_);
  }

private:
  void cb_odom(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    visualization_msgs::msg::Marker vel_msg;
    vel_msg.header.stamp = get_clock()->now();
    vel_msg.header.frame_id = odom_frame_id_;
    vel_msg.ns = "px4_gz";
    vel_msg.id = 0;
    vel_msg.type = visualization_msgs::msg::Marker::ARROW;
    vel_msg.action = visualization_msgs::msg::Marker::ADD;
    vel_msg.points.resize(2);
    auto &start_point = vel_msg.points[0];
    auto &end_point = vel_msg.points[1];
    start_point.x = msg->position[0];
    start_point.y = msg->position[1];
    start_point.z = msg->position[2];
    end_point = start_point;
    double speed =
        std::sqrt(msg->velocity[0] * msg->velocity[0] +
                  msg->velocity[1] * msg->velocity[1] +
                  msg->velocity[2] * msg->velocity[2]);
    double scale = std::min(speed, ARROW_MAX_LEN);
    end_point.x += msg->velocity[0] / speed * scale;
    end_point.y += msg->velocity[1] / speed * scale;
    end_point.z += msg->velocity[2] / speed * scale;
    vel_msg.scale.x = 0.1 * scale;
    vel_msg.scale.y = 0.2 * scale;
    vel_msg.scale.z = 0.2 * scale;
    vel_msg.color.a = 1.0;
    vel_msg.color.r = 1.0;
    vel_msg.color.g = 1.0;
    vel_msg.color.b = 0.0;
    pub_velocity_->publish(vel_msg);
  }

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
      drone_msg.id = 1;
      is_created_msg = true;
    }
    drone_msg.header.stamp = get_clock()->now();
    pub_drone_->publish(drone_msg);
  }

  void pub_world_msg() {
    static bool is_created_msg = false;
    static visualization_msgs::msg::MarkerArray world_msg;
    if (!is_created_msg) {
      int id = 2;
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
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr sub_odom_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_velocity_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_world_;
  rclcpp::TimerBase::SharedPtr timer_50hz_;
  rclcpp::TimerBase::SharedPtr timer_1hz_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_pub_;
  std::string world_file_path_;
  std::string odom_frame_id_;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::visualization_helper)