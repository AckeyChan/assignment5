#include "mid360_mapping/mapping_node.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

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
  
  map_frame_ = declare_parameter<std::string>("map_frame", "map", startup);//地图坐标系
  body_frame_ = declare_parameter<std::string>("body_frame", "body", startup);//机体坐标系
  fastlio_world_frame_ = declare_parameter<std::string>(
    "fastlio_world_frame", "camera_init", startup);
  gravity_align_ = declare_parameter<bool>("gravity_align", true, startup);//是否使用IMU数据计算重力方向，进行点云的重力对齐
  gravity_sample_count_ = declare_parameter<int>("gravity_sample_count", 200, startup);//采样的IMU数据数量，用于计算重力方向
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
  if (gravity_align_ && (map_frame_.empty() || fastlio_world_frame_.empty() ||
    map_frame_ == fastlio_world_frame_ || gravity_sample_count_ < 1))
  {
    throw std::invalid_argument(
            "gravity alignment requires distinct non-empty map_frame/fastlio_world_frame "
            "and gravity_sample_count >= 1");
  }
  map_from_camera_basis_.setIdentity();//初始化旋转矩阵为单位矩阵，这是四元数旋转矩阵的基础
  static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

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
      get_logger(), "FAST-LIO map snapshots=%llu; raw input counts clouds=%llu imu=%llu; "
      "gravity samples=%d/%d, aligned=%s",
      static_cast<unsigned long long>(map_count_),
      static_cast<unsigned long long>(cloud_count_),
      static_cast<unsigned long long>(imu_count_),
      gravity_samples_, gravity_sample_count_, map_transform_ready_ ? "yes" : "no");
  });
  RCLCPP_INFO(
    get_logger(), "FAST-LIO inputs: lidar %s%s, IMU %s%s; map input %s",
    fastlio_lidar_topic.c_str(), lidar_topic == fastlio_lidar_topic ? " (direct)" : " (relayed)",
    fastlio_imu_topic.c_str(), imu_topic == fastlio_imu_topic ? " (direct)" : " (relayed)",
    fastlio_map_topic.c_str());
  RCLCPP_INFO(
    get_logger(), "Gravity alignment %s; publishing in frame %s (FAST-LIO frame %s)",
    gravity_align_ ? "enabled" : "disabled", map_frame_.c_str(), fastlio_world_frame_.c_str());
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
  update_gravity_alignment(message);
}

void MappingNode::update_gravity_alignment(sensor_msgs::msg::Imu::ConstSharedPtr message)
{
  if (!gravity_align_ || gravity_ready_) {//如果不需要重力对齐或者已经完成重力对齐，就直接返回
    return;
  }
  gravity_accel_sum_[0] += message->linear_acceleration.x;
  gravity_accel_sum_[1] += message->linear_acceleration.y;
  gravity_accel_sum_[2] += message->linear_acceleration.z;
  ++gravity_samples_;
  if (gravity_samples_ < gravity_sample_count_) {
    return;
  }

  tf2::Vector3 up(
    gravity_accel_sum_[0] / gravity_samples_,
    gravity_accel_sum_[1] / gravity_samples_,
    gravity_accel_sum_[2] / gravity_samples_);
  if (up.length() < 1e-6) {
    RCLCPP_WARN(get_logger(), "Zero mean acceleration; cannot estimate gravity, resetting samples");
    gravity_samples_ = 0;
    gravity_accel_sum_[0] = gravity_accel_sum_[1] = gravity_accel_sum_[2] = 0.0;
    return;
  }
  up.normalize();

  const tf2::Vector3 world_up(0.0, 0.0, 1.0);
  const double dot = std::clamp(up.dot(world_up), -1.0, 1.0);//计算重力方向和世界坐标系的z轴方向的夹角余弦值
  if (dot > 1.0 - 1e-6) {//如果重力方向和世界坐标系的z轴方向一致，就不需要旋转
    map_from_camera_rotation_.setValue(0.0, 0.0, 0.0, 1.0);
  } else if (dot < -1.0 + 1e-6) {//如果重力方向和世界坐标系的z轴方向相反，就旋转180度
    map_from_camera_rotation_.setRotation(tf2::Vector3(1.0, 0.0, 0.0), M_PI);
  } else {//否则就计算旋转轴和旋转角度
    tf2::Vector3 axis = up.cross(world_up);
    axis.normalize();
    map_from_camera_rotation_.setRotation(axis, std::acos(dot));//计算旋转四元数
  }
  map_from_camera_rotation_.normalize();
  map_from_camera_basis_.setRotation(map_from_camera_rotation_);
  gravity_ready_ = true;

  RCLCPP_INFO(
    get_logger(), "Gravity from %d IMU samples: up=(%.3f, %.3f, %.3f), tilt %.1f deg",
    gravity_samples_, up.x(), up.y(), up.z(), std::acos(dot) * 180.0 / M_PI);
}

bool MappingNode::output_ready() const
{
  return !gravity_align_ || map_transform_ready_;
}

tf2::Vector3 MappingNode::transform_point(const tf2::Vector3 & point) const
{//将点云坐标系下的点转换到地图坐标系下
  return map_from_camera_basis_ * point + map_from_camera_translation_;
}

tf2::Quaternion MappingNode::transform_orientation(const tf2::Quaternion & orientation) const
{//将点云坐标系下的四元数转换到地图坐标系下
  tf2::Quaternion normalized = orientation;
  normalized.normalize();
  tf2::Quaternion result = map_from_camera_rotation_ * normalized;
  result.normalize();
  return result;
}
//
void MappingNode::publish_static_transform()
{
  if (!gravity_align_) {
    return;
  }
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = now();
  transform.header.frame_id = map_frame_;
  transform.child_frame_id = fastlio_world_frame_;
  transform.transform.translation.x = map_from_camera_translation_.x();
  transform.transform.translation.y = map_from_camera_translation_.y();
  transform.transform.translation.z = map_from_camera_translation_.z();
  transform.transform.rotation.x = map_from_camera_rotation_.x();
  transform.transform.rotation.y = map_from_camera_rotation_.y();
  transform.transform.rotation.z = map_from_camera_rotation_.z();
  transform.transform.rotation.w = map_from_camera_rotation_.w();
  static_tf_broadcaster_->sendTransform(transform);
}//发布静态变换，将地图坐标系下的点转换到点云坐标系下

bool MappingNode::transform_map_cloud(
  const sensor_msgs::msg::PointCloud2 & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  output = input;
  try {
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");//迭代器
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
    std::vector<float> z_values;
    if (!map_transform_ready_) {
      z_values.reserve(static_cast<std::size_t>(output.width) * output.height);
    }

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {//遍历点云中的每个点，并且将z轴的值保存到z_values中，用于计算地面高度
      if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
        continue;
      }
      const tf2::Vector3 rotated = map_from_camera_basis_ *
        tf2::Vector3(*iter_x, *iter_y, *iter_z);//将点云坐标系下的点旋转到地图坐标系下
      *iter_x = static_cast<float>(rotated.x());
      *iter_y = static_cast<float>(rotated.y());
      *iter_z = static_cast<float>(rotated.z());
      if (!map_transform_ready_) {
        z_values.push_back(static_cast<float>(rotated.z()));
      }
    }

    if (!map_transform_ready_) {
      if (z_values.empty()) {
        RCLCPP_WARN(get_logger(), "FAST-LIO map has no finite points; cannot estimate ground level");
        return false;
      }
      const std::size_t index = std::min(z_values.size() - 1, z_values.size() / 100);//找到1%的点的高度作为地面高度
      std::nth_element(z_values.begin(), z_values.begin() + index, z_values.end());
      const double ground_z = z_values[index];
      map_from_camera_translation_.setZ(-ground_z);
      map_transform_ready_ = true;
      publish_static_transform();//
      RCLCPP_INFO(
        get_logger(), "Ground level from first map: z=%.3f m; map frame origin set to ground",
        ground_z);
    }

    sensor_msgs::PointCloud2Iterator<float> shift_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> shift_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> shift_z(output, "z");
    for (; shift_x != shift_x.end(); ++shift_x, ++shift_y, ++shift_z) {//遍历点云中的每个点，并且将z轴的值加上地面高度，用于将点云坐标系下的点转换到地图坐标系下
      if (!std::isfinite(*shift_x) || !std::isfinite(*shift_y) || !std::isfinite(*shift_z)) {
        continue;
      }
      *shift_x += static_cast<float>(map_from_camera_translation_.x());//将点云坐标系下的点转换到地图坐标系下
      *shift_y += static_cast<float>(map_from_camera_translation_.y());
      *shift_z += static_cast<float>(map_from_camera_translation_.z());
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Cannot transform FAST-LIO map: %s", error.what());
    return false;
  }

  output.header.frame_id = map_frame_;
  return true;
}

void MappingNode::on_fastlio_map(sensor_msgs::msg::PointCloud2::ConstSharedPtr message)
{
  ++map_count_;
  if (gravity_align_ && !gravity_ready_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for gravity alignment before publishing FAST-LIO output");
    return;
  }

  sensor_msgs::msg::PointCloud2 output;
  if (gravity_align_) {
    if (!transform_map_cloud(*message, output)) {
      return;
    }
  } else {
    output = *message;
  }

  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    latest_map_ = std::make_shared<sensor_msgs::msg::PointCloud2>(output);
  }
  map_pub_->publish(output);
}

void MappingNode::on_fastlio_odom(nav_msgs::msg::Odometry::ConstSharedPtr message)
{//将FAST-LIO的里程计消息转换到地图坐标系下，并发布
  if (!output_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for gravity alignment before publishing FAST-LIO odometry");
    return;
  }
  if (!gravity_align_) {
    odom_pub_->publish(*message);
    return;
  }

  nav_msgs::msg::Odometry output = *message;
  output.header.frame_id = map_frame_;
  if (!body_frame_.empty()) {
    output.child_frame_id = body_frame_;
  }
  const auto & position = message->pose.pose.position;
  const tf2::Vector3 transformed = transform_point(
    tf2::Vector3(position.x, position.y, position.z));
  output.pose.pose.position.x = transformed.x();
  output.pose.pose.position.y = transformed.y();
  output.pose.pose.position.z = transformed.z();
  const auto & orientation = message->pose.pose.orientation;
  const tf2::Quaternion transformed_orientation = transform_orientation(
    tf2::Quaternion(orientation.x, orientation.y, orientation.z, orientation.w));
  output.pose.pose.orientation.x = transformed_orientation.x();
  output.pose.pose.orientation.y = transformed_orientation.y();
  output.pose.pose.orientation.z = transformed_orientation.z();
  output.pose.pose.orientation.w = transformed_orientation.w();
  odom_pub_->publish(output);
}

void MappingNode::on_fastlio_path(nav_msgs::msg::Path::ConstSharedPtr message)
{//将FAST-LIO的路径消息转换到地图坐标系下，并发布
  if (!output_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Waiting for gravity alignment before publishing FAST-LIO path");
    return;
  }
  if (!gravity_align_) {
    path_pub_->publish(*message);
    return;
  }

  nav_msgs::msg::Path output = *message;
  output.header.frame_id = map_frame_;
  for (auto & pose : output.poses) {
    pose.header.frame_id = map_frame_;
    const tf2::Vector3 transformed = transform_point(
      tf2::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
    pose.pose.position.x = transformed.x();
    pose.pose.position.y = transformed.y();
    pose.pose.position.z = transformed.z();
    const tf2::Quaternion transformed_orientation = transform_orientation(
      tf2::Quaternion(
        pose.pose.orientation.x, pose.pose.orientation.y,
        pose.pose.orientation.z, pose.pose.orientation.w));
    pose.pose.orientation.x = transformed_orientation.x();
    pose.pose.orientation.y = transformed_orientation.y();
    pose.pose.orientation.z = transformed_orientation.z();
    pose.pose.orientation.w = transformed_orientation.w();
  }
  path_pub_->publish(output);
}
}  // namespace mid360_mapping
