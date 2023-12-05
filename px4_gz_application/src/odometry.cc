#include <Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/static_transform_broadcaster.h>

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
    pub_odom_ = create_publisher<px4_msgs::msg::VehicleOdometry>("pub/odom", 10);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    load_param();
    init_callback();
  }

private:
  void load_param() {
    odom_frame_id_ = declare_parameter("odom_frame_id", "odom");
    base_link_frame_id_ = declare_parameter("base_link_frame_id", "base_link");
    declare_parameter("yaw_frd_ned", 0.0);
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
      this->set_parameter({"yaw_frd_ned", yaw});
      this->timer_1hz_ =
          this->create_wall_timer(1s, std::bind(&odometry::cb_1hz, this));
      RCLCPP_INFO_STREAM(
          get_logger(),
          "detect init yaw(degree) in odom_ned frame: " << yaw / M_PI * 180.0);
    });
  }

  void cb_px4_odometry(px4_msgs::msg::VehicleOdometry::ConstSharedPtr msg) {
    last_msg_ = msg;
    px4_msgs::msg::VehicleOdometry new_msg = *msg;
    upate_position_orientation(new_msg);
    pub_odom_->publish(new_msg);
  }

  void upate_position_orientation(px4_msgs::msg::VehicleOdometry &msg) {
    static rclcpp::Rate rate(1);
    static Eigen::Isometry3d T_localflu_odomflu;
    do {
      try {
        T_localflu_odomflu = tf2::transformToEigen(tf_buffer_->lookupTransform(
            odom_frame_id_, base_link_frame_id_, tf2::TimePointZero));
        break;
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN_STREAM(this->get_logger(), "waiting for transform from "
                                                   << base_link_frame_id_
                                                   << " to " << odom_frame_id_);
      }
    } while (rate.sleep());
    auto T_localfrd_odomned = T_odomflu_odomned_ * T_localflu_odomflu * T_localfrd_localflu_;
    auto p = T_localfrd_odomned.translation().cast<float>();
    Eigen::Quaternionf q(T_localfrd_odomned.linear().cast<float>());
    msg.position = {p.x(), p.y(), p.z()};
    msg.q = {q.w(), q.x(), q.y(), q.z()};
  }

  void cb_1hz() {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = get_clock()->now();
    msg.transform.translation = tf2::toMsg2(T_odomned_odomflu_.translation());
    msg.transform.rotation = tf2::toMsg(Eigen::Quaterniond{T_odomned_odomflu_.rotation()});
    msg.header.frame_id = odom_frame_id_;
    msg.child_frame_id = odom_frame_id_ + "_ned";
    tf_static_broadcaster_->sendTransform(msg);

    msg.transform.translation = tf2::toMsg2(T_localfrd_localflu_.translation());
    msg.transform.rotation = tf2::toMsg(Eigen::Quaterniond{T_localfrd_localflu_.rotation()});
    msg.header.frame_id = base_link_frame_id_;
    msg.child_frame_id = base_link_frame_id_ + "_frd";
    tf_static_broadcaster_->sendTransform(msg);
  }

private:
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr
      sub_vehicle_odometry_ = nullptr;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr pub_odom_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_once_10s_ = nullptr;
  rclcpp::TimerBase::SharedPtr timer_1hz_ = nullptr;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ = nullptr;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ = nullptr;
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  Eigen::Isometry3d T_odomflu_odomned_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_odomned_odomflu_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_localflu_localfrd_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d T_localfrd_localflu_ = Eigen::Isometry3d::Identity();
  px4_pose cur_pose_;
  px4_msgs::msg::VehicleOdometry::ConstSharedPtr last_msg_ = nullptr;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ = nullptr;
};
}; // namespace px4_gz

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(px4_gz::odometry)