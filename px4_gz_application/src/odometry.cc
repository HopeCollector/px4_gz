#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace px4_gz {
struct px4_pose {
  uint64_t stamp; // nanosec
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d linear_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d angle_vel = Eigen::Vector3d::Zero();
};

inline double quaternion_to_yaw(Eigen::Quaterniond q) {
  Eigen::Vector3d vec = q * Eigen::Vector3d::UnitX();
  return std::atan2(vec[1], vec[0]);
}

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
    sub_vehicle_odometry_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(),
        std::bind(&odometry::cb_px4_odometry, this, _1));
    timer_once_10s_ = create_wall_timer(10s, [this]() {
      this->timer_once_10s_->cancel();
      double yaw = quaternion_to_yaw(Eigen::Quaterniond(
          last_msg_->q[0], last_msg_->q[1], last_msg_->q[2], last_msg_->q[3]));
      this->T_odomflu_odomned_.linear() =
          Eigen::Matrix3d(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()));
      this->T_odomned_odomflu_ = this->T_odomflu_odomned_.inverse();
      this->T_localflu_localfrd_.linear() =
          Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).toRotationMatrix();
      this->T_localfrd_localflu_ = this->T_localflu_localfrd_.inverse();
      RCLCPP_INFO_STREAM(
          get_logger(),
          "detect init yaw(degree) in odom_ned frame: " << yaw / M_PI * 180.0);
    });
  }

  void cb_px4_odometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    // Eigen::Affine3d T_localfrd_odomned = Eigen::Affine3d::Identity();
    // T_localfrd_odomned.translation() << msg->position[0], msg->position[1],
    //     msg->position[2];
    // T_localfrd_odomned.linear() =
    //     Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3])
    //         .toRotationMatrix();
    // Eigen::Affine3d T_localflu_odomflu =
    //     T_odomned_odomflu_ * T_localfrd_odomned * T_localflu_localfrd_;
    // cur_pose_.position = T_localflu_odomflu.translation();
    // cur_pose_.orientation = T_localflu_odomflu.linear();
    last_msg_ = msg;
    upate_position_orientation();
    cur_pose_.linear_vel << msg->velocity[0], msg->velocity[1],
        msg->velocity[2];
    cur_pose_.angle_vel << msg->angular_velocity[0], msg->angular_velocity[1],
        msg->angular_velocity[2];
    cur_pose_.linear_vel = cur_pose_.orientation.conjugate() * T_odomned_odomflu_ *
                           cur_pose_.linear_vel;
    cur_pose_.angle_vel =
        cur_pose_.orientation.conjugate() * T_odomned_odomflu_ * cur_pose_.angle_vel;
    cur_pose_.stamp = msg->timestamp * 1000; // us -> ns
    publish_odom_msg();
  }

  void upate_position_orientation() {
    static geometry_msgs::msg::TransformStamped transform;
    static rclcpp::Rate rate(1);
    do {
      try {
        transform = tf_buffer_->lookupTransform(
            odom_frame_id_, base_link_frame_id_, tf2::TimePointZero);
        break;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "waiting for transform from "
                                                   << base_link_frame_id_
                                                   << " to " << odom_frame_id_);
      }
    } while (rate.sleep());
    cur_pose_.position[0] = transform.transform.translation.x;
    cur_pose_.position[1] = transform.transform.translation.y;
    cur_pose_.position[2] = transform.transform.translation.z;
    cur_pose_.orientation.w() = transform.transform.rotation.w;
    cur_pose_.orientation.x() = transform.transform.rotation.x;
    cur_pose_.orientation.y() = transform.transform.rotation.y;
    cur_pose_.orientation.z() = transform.transform.rotation.z;
  }

  void publish_odom_msg() {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = get_clock()->now();
    odom.header.frame_id = odom_frame_id_;
    odom.child_frame_id = base_link_frame_id_;
    odom.pose.pose.position.x = cur_pose_.position[0];
    odom.pose.pose.position.y = cur_pose_.position[1];
    odom.pose.pose.position.z = cur_pose_.position[2];
    odom.pose.pose.orientation.w = cur_pose_.orientation.w();
    odom.pose.pose.orientation.x = cur_pose_.orientation.x();
    odom.pose.pose.orientation.y = cur_pose_.orientation.y();
    odom.pose.pose.orientation.z = cur_pose_.orientation.z();
    odom.twist.twist.linear.x = cur_pose_.linear_vel[0];
    odom.twist.twist.linear.y = cur_pose_.linear_vel[1];
    odom.twist.twist.linear.z = cur_pose_.linear_vel[2];
    odom.twist.twist.angular.x = cur_pose_.angle_vel[0];
    odom.twist.twist.angular.y = cur_pose_.angle_vel[1];
    odom.twist.twist.angular.z = cur_pose_.angle_vel[2];
    pub_odom_->publish(odom);
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      sub_vehicle_odometry_ = nullptr;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_once_10s_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  Eigen::Affine3d T_odomflu_odomned_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_odomned_odomflu_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localflu_localfrd_ = Eigen::Affine3d::Identity();
  Eigen::Affine3d T_localfrd_localflu_ = Eigen::Affine3d::Identity();
  px4_pose cur_pose_;
  px4_msgs::msg::VehicleOdometry::ConstSharedPtr last_msg_ = nullptr;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::odometry)