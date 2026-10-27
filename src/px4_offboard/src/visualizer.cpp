#include <cmath>
#include <cstddef>
#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;

class PX4Visualizer : public rclcpp::Node
{
public:
  PX4Visualizer()
  : Node("px4_visualizer")
  {
    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    const auto visualizer_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "~/markers", visualizer_qos);
    vehicle_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/vehicle_path", visualizer_qos);
    setpoint_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "~/setpoint_path", visualizer_qos);

    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      "/fmu/out/vehicle_attitude", px4_qos,
      [this](const px4_msgs::msg::VehicleAttitude::SharedPtr msg) {
        vehicle_attitude_callback(msg);
      });
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position", px4_qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        vehicle_local_position_callback(msg);
      });
    trajectory_setpoint_sub_ =
      create_subscription<px4_msgs::msg::TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", px4_qos,
      [this](const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg) {
        trajectory_setpoint_callback(msg);
      });

    timer_ = create_wall_timer(50ms, std::bind(&PX4Visualizer::cmdloop_callback, this));
    RCLCPP_INFO(
      get_logger(),
      "PX4 visualizer started. Add /px4_visualizer/markers and paths in RViz.");
  }

private:
  static constexpr double kPi = 3.14159265358979323846;
  static constexpr const char * kFrameId = "map";
  static constexpr std::size_t kMaxPathSize = 2000;

  geometry_msgs::msg::Pose vector2PoseMsg(float x, float y, float z, double yaw) const
  {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.z = std::sin(yaw / 2.0);
    pose.orientation.w = std::cos(yaw / 2.0);
    return pose;
  }

  void vehicle_attitude_callback(
    const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    const auto & q = msg->q;
    const double yaw_ned = std::atan2(
      2.0 * (q[0] * q[3] + q[1] * q[2]),
      1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
    vehicle_yaw_ = kPi / 2.0 - yaw_ned;
  }

  void vehicle_local_position_callback(
    const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg)
  {
    if (!msg->xy_valid || !msg->z_valid) {
      return;
    }

    // PX4 local position is NED (x north, y east, z down); RViz uses ENU.
    current_pose_ = vector2PoseMsg(msg->x, msg->y, -msg->z, vehicle_yaw_);
    position_received_ = true;
  }

  void trajectory_setpoint_callback(
    const px4_msgs::msg::TrajectorySetpoint::SharedPtr msg)
  {
    if (!std::isfinite(msg->position[0]) ||
      !std::isfinite(msg->position[1]) ||
      !std::isfinite(msg->position[2]))
    {
      return;
    }

    const double yaw = std::isfinite(msg->yaw) ? kPi / 2.0 - msg->yaw : vehicle_yaw_;
    target_pose_ = vector2PoseMsg(
      msg->position[0], msg->position[1], -msg->position[2], yaw);
    setpoint_received_ = true;
  }

  visualization_msgs::msg::Marker create_arrow_marker(
    const geometry_msgs::msg::Pose & pose, int id, float r, float g, float b) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = kFrameId;
    marker.header.stamp = now();
    marker.ns = "px4";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose;
    marker.scale.x = 1.5;
    marker.scale.y = 0.18;
    marker.scale.z = 0.18;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;
    return marker;
  }

  void append_vehicle_path(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = kFrameId;
    pose.header.stamp = stamp;
    pose.pose = current_pose_;
    vehicle_path_.header = pose.header;
    vehicle_path_.poses.push_back(pose);
    if (vehicle_path_.poses.size() > kMaxPathSize) {
      vehicle_path_.poses.erase(vehicle_path_.poses.begin());
    }
  }

  void append_setpoint_path(const rclcpp::Time & stamp)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = kFrameId;
    pose.header.stamp = stamp;
    pose.pose = target_pose_;
    setpoint_path_.header = pose.header;
    setpoint_path_.poses.push_back(pose);
    if (setpoint_path_.poses.size() > kMaxPathSize) {
      setpoint_path_.poses.erase(setpoint_path_.poses.begin());
    }
  }

  void cmdloop_callback()
  {
    if (!position_received_) {
      return;
    }

    const auto stamp = now();
    append_vehicle_path(stamp);
    if (setpoint_received_) {
      append_setpoint_path(stamp);
    }

    marker_pub_->publish(create_arrow_marker(current_pose_, 0, 0.1F, 0.8F, 1.0F));
    if (setpoint_received_) {
      auto target_marker = create_arrow_marker(target_pose_, 1, 1.0F, 0.7F, 0.1F);
      target_marker.scale.x = 0.8;
      marker_pub_->publish(target_marker);
    }
    vehicle_path_pub_->publish(vehicle_path_);
    if (setpoint_received_) {
      setpoint_path_pub_->publish(setpoint_path_);
    }
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr vehicle_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr setpoint_path_pub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr
    local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::TrajectorySetpoint>::SharedPtr
    trajectory_setpoint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Pose current_pose_{};
  geometry_msgs::msg::Pose target_pose_{};
  nav_msgs::msg::Path vehicle_path_{};
  nav_msgs::msg::Path setpoint_path_{};
  double vehicle_yaw_{0.0};
  bool position_received_{false};
  bool setpoint_received_{false};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PX4Visualizer>());
  rclcpp::shutdown();
  return 0;
}
