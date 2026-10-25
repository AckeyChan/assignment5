#include "mid360_mapping/mapping_node.hpp"
#include <chrono>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace mid360_mapping
{
MappingNode::MappingNode() : Node("mapping_node")
{
  rcl_interfaces::msg::ParameterDescriptor startup;
  startup.read_only = true;
  startup.description = "Startup setting; restart the node after changing configuration";
  const auto input_type = declare_parameter<std::string>("input_type", "pointcloud2", startup);
  const auto lidar_topic = declare_parameter<std::string>("lidar_topic", "/livox/lidar", startup);//从什么话题接收点云数据
  const auto imu_topic = declare_parameter<std::string>("imu_topic", "/livox/imu", startup);//从什么话题接收IMU数据
  const auto fastlio_lidar_topic = declare_parameter<std::string>(
    "fastlio_lidar_topic", "/livox/lidar", startup);//FAST-LIO的接收点云话题
  const auto fastlio_imu_topic = declare_parameter<std::string>(
    "fastlio_imu_topic", "/livox/imu", startup);//FAST-LIO的接收IMU话题
  const auto fastlio_map_topic = declare_parameter<std::string>("fastlio_map_topic", "/Laser_map", startup);
  const auto fastlio_odom_topic = declare_parameter<std::string>("fastlio_odom_topic", "/Odometry", startup);
  const auto fastlio_path_topic = declare_parameter<std::string>("fastlio_path_topic", "/path", startup);
  
  declare_parameter<std::string>("map_frame", "map", startup);
  declare_parameter<std::string>("body_frame", "body", startup);
  map_output_path_ = declare_parameter<std::string>("map_output_path", "maps/fastlio_map.pcd", startup);
  if (lidar_topic.empty() || imu_topic.empty()) {//关键topic检查
    throw std::invalid_argument("lidar_topic and imu_topic must not be empty");
  }
  if (fastlio_map_topic.empty() || fastlio_odom_topic.empty() || fastlio_path_topic.empty()) {
    throw std::invalid_argument("FAST-LIO output topics must not be empty");
  }
  if (fastlio_lidar_topic.empty() || fastlio_imu_topic.empty()) {
    throw std::invalid_argument("FAST-LIO input topics must not be empty");
  }

  if (input_type == "pointcloud2") {
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic, rclcpp::SensorDataQoS().keep_last(10),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_cloud(msg);});
    if (lidar_topic != fastlio_lidar_topic) {//如果输入的点云话题和FAST-LIO的点云话题不一样，就创建一个发布器，把接收到的点云消息转发到FAST-LIO的点云话题
      fastlio_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        fastlio_lidar_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    }
  } else if (input_type == "livox_custom") {
    // A generic subscription avoids a compile-time Livox SDK dependency in the scaffold.
    // The livox_ros_driver2 message type support must still be installed and sourced.
    custom_sub_ = create_generic_subscription(
      lidar_topic, "livox_ros_driver2/msg/CustomMsg", rclcpp::SensorDataQoS().keep_last(10),
      [this](std::shared_ptr<rclcpp::SerializedMessage> msg) {on_custom_cloud(msg);});
    if (lidar_topic != fastlio_lidar_topic) {//如果输入的点云话题和FAST-LIO的点云话题不一样，就创建一个发布器，把接收到的点云消息转发到FAST-LIO的点云话题
      fastlio_custom_pub_ = create_generic_publisher(
        fastlio_lidar_topic, "livox_ros_driver2/msg/CustomMsg",
        rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
    }
  } else {
    throw std::invalid_argument("input_type must be pointcloud2 or livox_custom");
  }
  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::SensorDataQoS().keep_last(1000),
    [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {on_imu(msg);});
  if (imu_topic != fastlio_imu_topic) {//如果输入的IMU话题和FAST-LIO的IMU话题不一样，就创建一个发布器，把接收到的IMU消息转发到FAST-LIO的IMU话题
    fastlio_imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(
      fastlio_imu_topic, rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  }

  fastlio_map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    fastlio_map_topic, rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {on_fastlio_map(msg);});
  fastlio_odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    fastlio_odom_topic, 10,
    [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {on_fastlio_odom(msg);});
  fastlio_path_sub_ = create_subscription<nav_msgs::msg::Path>(
    fastlio_path_topic, 10,
    [this](nav_msgs::msg::Path::ConstSharedPtr msg) {on_fastlio_path(msg);});

  // These are output contracts, not an implemented mapping pipeline.
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("mapping/map", rclcpp::SensorDataQoS());
  registered_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "mapping/cloud_registered", rclcpp::SensorDataQoS());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("mapping/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("mapping/path", 10);
  save_service_ = create_service<std_srvs::srv::Trigger>(
    "mapping/save_map",
    [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
      if (map_output_path_.empty()) {
        response->success = false;
        response->message = "map_output_path is empty";
        return;
      }

      sensor_msgs::msg::PointCloud2::ConstSharedPtr map;
      {
        std::lock_guard<std::mutex> lock(map_mutex_);
        map = latest_map_;
      }
      if (!map || map->data.empty() || map->width == 0 || map->height == 0) {
        response->success = false;
        response->message = "No FAST-LIO map has been received";
        return;
      }

      const std::filesystem::path output_path(map_output_path_);
      const auto parent = output_path.parent_path();
      if (!parent.empty()) {
        std::error_code directory_error;
        if (!std::filesystem::is_directory(parent, directory_error)) {
          response->success = false;
          response->message = directory_error ?
            "Cannot access output directory: " + directory_error.message() :
            "Output directory does not exist: " + parent.string();
          return;
        }
      }

      try {
        pcl::PCLPointCloud2 pcl_map;
        pcl_conversions::toPCL(*map, pcl_map);
        pcl::PCDWriter writer;
        const auto result = writer.writeBinary(map_output_path_, pcl_map);
        response->success = result == 0;
        response->message = result == 0 ?
          "Saved latest FAST-LIO map to " + map_output_path_ :
          "Failed to write PCD to " + map_output_path_;
      } catch (const std::exception & error) {
        response->success = false;
        response->message = std::string("Failed to save PCD: ") + error.what();
      }
    });
  status_timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
    RCLCPP_INFO(
      get_logger(), "FAST-LIO map snapshots=%llu; raw input counts clouds=%llu imu=%llu",
      static_cast<unsigned long long>(map_count_),
      static_cast<unsigned long long>(cloud_count_),
      static_cast<unsigned long long>(imu_count_));
  });
  RCLCPP_INFO(
    get_logger(), "FAST-LIO inputs: lidar %s%s, IMU %s%s; map input %s",
    fastlio_lidar_topic.c_str(), lidar_topic == fastlio_lidar_topic ? " (direct)" : " (relayed)",
    fastlio_imu_topic.c_str(), imu_topic == fastlio_imu_topic ? " (direct)" : " (relayed)",
    fastlio_map_topic.c_str());
}

void MappingNode::on_cloud(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++cloud_count_;
  if (fastlio_cloud_pub_) {
    fastlio_cloud_pub_->publish(*message);
  }
  // TODO: Read actual point fields and units, including per-point acquisition times.
  // TODO: Connect to your chosen mapping backend; preserve sensor timestamps and frames.
}

void MappingNode::on_custom_cloud(std::shared_ptr<rclcpp::SerializedMessage> message)
{
  ++cloud_count_;
  if (fastlio_custom_pub_) {
    fastlio_custom_pub_->publish(*message);
  }
  // TODO: Replace serialized-message reception with a typed Livox input adapter.
  // Declare livox_ros_driver2 in package.xml and CMake when using its C++ message type.
  // TODO: Preserve timebase, offset_time, reflectivity, tag and line as needed by your backend.
}

void MappingNode::on_imu(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  ++imu_count_;
  if (fastlio_imu_pub_) {
    fastlio_imu_pub_->publish(*message);
  }
  // TODO: Check the recorded units/frame and handle the IMU data in your mapping backend.
  // TODO: Publish estimated poses, transforms, registered clouds and an accumulated map.
  // Counts alone are not synchronization, deskewing or mapping.
}

void MappingNode::on_fastlio_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = message;
  }
  ++map_count_;
  map_pub_->publish(*message);
}

void MappingNode::on_fastlio_odom(nav_msgs::msg::Odometry::ConstSharedPtr message)
{
  odom_pub_->publish(*message);
}

void MappingNode::on_fastlio_path(nav_msgs::msg::Path::ConstSharedPtr message)
{
  path_pub_->publish(*message);
}
}  // namespace mid360_mapping
