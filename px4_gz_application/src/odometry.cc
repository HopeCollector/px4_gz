#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace px4_gz {
class odometry : public rclcpp::Node {
public:
  explicit odometry(const rclcpp::NodeOptions &options)
      : Node("odometry", options) {
    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("pub/odom", 10);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    load_param();
    init_callback();
  }

private:
  void load_param() {
    odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
    base_link_frame_id_ = declare_parameter("base_link_frame_id", "base_link");
  }

  void init_callback() {
    using namespace std::chrono_literals;
    using namespace std::placeholders;
    timer_100hz_ = create_wall_timer(10ms, std::bind(&odometry::cb_100hz, this));
  }

  void cb_100hz() {
    static geometry_msgs::msg::TransformStamped transform;
    static rclcpp::Rate rate(0.5);
    try {
      transform = tf_buffer_->lookupTransform(
          odom_frame_id_, base_link_frame_id_, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN_STREAM(this->get_logger(), "waiting for transform from "
                                                 << base_link_frame_id_
                                                 << " to " << odom_frame_id_);
      rate.sleep();
    }
    nav_msgs::msg::Odometry odom;
    odom.header = transform.header;
    odom.child_frame_id = base_link_frame_id_;
    odom.pose.pose.position.x = transform.transform.translation.x;
    odom.pose.pose.position.y = transform.transform.translation.y;
    odom.pose.pose.position.z = transform.transform.translation.z;
    odom.pose.pose.orientation = transform.transform.rotation;
    pub_odom_->publish(odom);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::TimerBase::SharedPtr timer_100hz_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::odometry)