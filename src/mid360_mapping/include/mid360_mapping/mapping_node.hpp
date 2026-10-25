#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace mid360_mapping
{
class MappingNode : public rclcpp::Node
{
public:
  MappingNode();

private:
  void on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_custom_cloud(std::shared_ptr<rclcpp::SerializedMessage> message);
  void on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message);
  void on_fastlio_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message);
  void on_fastlio_odom(nav_msgs::msg::Odometry::ConstSharedPtr message);
  void on_fastlio_path(nav_msgs::msg::Path::ConstSharedPtr message);

  void update_gravity_alignment(sensor_msgs::msg::Imu::ConstSharedPtr message);
  bool transform_map_cloud(
    const sensor_msgs::msg::PointCloud2 & input,
    sensor_msgs::msg::PointCloud2 & output);
  bool output_ready() const;
  void publish_static_transform();
  tf2::Vector3 transform_point(const tf2::Vector3 & point) const;
  tf2::Quaternion transform_orientation(const tf2::Quaternion & orientation) const;

  std::uint64_t cloud_count_{0};
  std::uint64_t imu_count_{0};
  std::uint64_t map_count_{0};
  std::string map_output_path_;
  std::string map_frame_;
  std::string body_frame_;
  std::string fastlio_world_frame_;
  bool gravity_align_{true};
  int gravity_sample_count_{200};
  int gravity_samples_{0};
  double gravity_accel_sum_[3]{0.0, 0.0, 0.0};
  bool gravity_ready_{false};
  bool map_transform_ready_{false};
  tf2::Quaternion map_from_camera_rotation_{0.0, 0.0, 0.0, 1.0};
  tf2::Matrix3x3 map_from_camera_basis_;
  tf2::Vector3 map_from_camera_translation_{0.0, 0.0, 0.0};
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::mutex map_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_map_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr fastlio_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr fastlio_odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr fastlio_path_sub_;
  rclcpp::GenericSubscription::SharedPtr custom_sub_;
  rclcpp::GenericPublisher::SharedPtr fastlio_custom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fastlio_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr fastlio_imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr registered_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};
}  // namespace mid360_mapping
