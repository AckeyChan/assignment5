#include <chrono>
#include <algorithm>
#include <cstdint>
#include <exception>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <memory>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_srvs/srv/trigger.hpp>

class MapProcessorNode : public rclcpp::Node
{
public:
  MapProcessorNode() : Node("map_processor_node")
  {//基本的参数声明和检查
    const auto input = declare_parameter<std::string>("input_topic", "/mapping/map");
    declare_parameter<std::string>("map_frame", "map");
    output_directory_ = declare_parameter<std::string>("output_directory", "maps");
    voxel_size_ = declare_parameter<double>("voxel_size", 0.10);
    grid_resolution_ = declare_parameter<double>("grid_resolution", 0.10);
    // These are starting values to tune, not measured ground/obstacle thresholds.
    obstacle_height_min_ = declare_parameter<double>("obstacle_height_min", 0.15);
    obstacle_height_max_ = declare_parameter<double>("obstacle_height_max", 1.50);
    //两种滤波器参数
    sor_enabled_ = declare_parameter<bool>("sor_enabled", false);
    sor_mean_k_ = declare_parameter<int>("sor_mean_k", 0);
    sor_stddev_multiplier_ = declare_parameter<double>("sor_stddev_multiplier", 0.0);
    radius_enabled_ = declare_parameter<bool>("radius_enabled", false);
    radius_search_ = declare_parameter<double>("radius_search", 0.0);
    radius_min_neighbors_ = declare_parameter<int>("radius_min_neighbors", 0);
    if (input.empty() || !std::isfinite(voxel_size_) || voxel_size_ <= 0 ||
      !std::isfinite(grid_resolution_) || grid_resolution_ <= 0 ||
      !std::isfinite(obstacle_height_min_) || !std::isfinite(obstacle_height_max_) ||
      obstacle_height_max_ <= obstacle_height_min_)
    {
      throw std::invalid_argument("Invalid map-processing startup parameters");
    }
    if ((sor_enabled_ && (sor_mean_k_ < 2 || !std::isfinite(sor_stddev_multiplier_) ||
      sor_stddev_multiplier_ < 0.0)) ||
      (radius_enabled_ && (!std::isfinite(radius_search_) || radius_search_ <= 0.0 ||
      radius_min_neighbors_ < 1)))
    {
      throw std::invalid_argument("Invalid outlier-filter parameters");
    }
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS(), [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        ++received_;
        auto cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(*msg, *cloud);
        last_input_points_ = static_cast<std::uint64_t>(cloud->width) * cloud->height;

        if (sor_enabled_) {//sor滤波器
          auto sor_filtered_cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
          pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor_filter;
          sor_filter.setInputCloud(cloud);
          sor_filter.setMeanK(sor_mean_k_);
          sor_filter.setStddevMulThresh(sor_stddev_multiplier_);
          sor_filter.filter(*sor_filtered_cloud);
          cloud = sor_filtered_cloud;
        }

        if (radius_enabled_) {//radius滤波器
          auto radius_filtered_cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);
          pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> radius_filter;
          radius_filter.setInputCloud(cloud);
          radius_filter.setRadiusSearch(radius_search_);
          radius_filter.setMinNeighborsInRadius(radius_min_neighbors_);
          radius_filter.filter(*radius_filtered_cloud);
          cloud = radius_filtered_cloud;
        }

        auto downsampled_cloud = pcl::PCLPointCloud2::Ptr(new pcl::PCLPointCloud2);//体素滤波器
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        const auto leaf_size = static_cast<float>(voxel_size_);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_filter.filter(*downsampled_cloud);
        cloud = downsampled_cloud;

        sensor_msgs::msg::PointCloud2 output;
        pcl_conversions::fromPCL(*cloud, output);
        output.header = msg->header;
        last_output_points_ = static_cast<std::uint64_t>(cloud->width) * cloud->height;
        cleaned_pub_->publish(output);

        pcl::PointCloud<pcl::PointXYZ> points;
        pcl::fromPCLPointCloud2(*cloud, points);
        bool have_finite_point = false;
        double min_x = 0.0;
        double min_y = 0.0;
        double max_x = 0.0;
        double max_y = 0.0;
        for (const auto & point : points) {//计算点云的边界框
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
          }
          if (!have_finite_point) {
            min_x = max_x = point.x;
            min_y = max_y = point.y;
            have_finite_point = true;
          } else {
            min_x = std::min(min_x, static_cast<double>(point.x));
            min_y = std::min(min_y, static_cast<double>(point.y));
            max_x = std::max(max_x, static_cast<double>(point.x));
            max_y = std::max(max_y, static_cast<double>(point.y));
          }
        }
        if (!have_finite_point) {
          RCLCPP_WARN(get_logger(), "Cannot create occupancy grid from an empty or invalid cloud");
          return;
        }

        const double width_cells = std::floor((max_x - min_x) / grid_resolution_) + 1.0;//计算栅格地图的宽度和高度
        const double height_cells = std::floor((max_y - min_y) / grid_resolution_) + 1.0;
        const double max_dimension = std::numeric_limits<std::uint32_t>::max();
        if (!std::isfinite(width_cells) || !std::isfinite(height_cells) ||
          width_cells < 1.0 || height_cells < 1.0 ||
          width_cells > max_dimension || height_cells > max_dimension)
        {
          RCLCPP_ERROR(get_logger(), "Occupancy grid dimensions are out of range");
          return;
        }

        nav_msgs::msg::OccupancyGrid grid;//写入栅格地图
        grid.header = msg->header;
        grid.info.map_load_time = msg->header.stamp;
        grid.info.resolution = static_cast<float>(grid_resolution_);
        grid.info.width = static_cast<std::uint32_t>(width_cells);
        grid.info.height = static_cast<std::uint32_t>(height_cells);
        grid.info.origin.position.x = min_x;
        grid.info.origin.position.y = min_y;
        grid.info.origin.orientation.w = 1.0;
        const auto cell_count = static_cast<std::uint64_t>(grid.info.width) * grid.info.height;
        if (cell_count > grid.data.max_size()) {
          RCLCPP_ERROR(get_logger(), "Occupancy grid is too large to allocate");
          return;
        }
        grid.data.assign(static_cast<std::size_t>(cell_count), 0);

        for (const auto & point : points) {//将点云投影到栅格地图上
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z) ||
            point.z < obstacle_height_min_ || point.z > obstacle_height_max_)
          {
            continue;
          }
          const auto column = static_cast<std::uint32_t>(
            std::floor((point.x - min_x) / grid_resolution_));
          const auto row = static_cast<std::uint32_t>(
            std::floor((point.y - min_y) / grid_resolution_));
          if (column < grid.info.width && row < grid.info.height) {
            const auto index = static_cast<std::size_t>(row) * grid.info.width + column;
            grid.data[index] = 100;
          }
        }
        grid_pub_->publish(grid);
        {
          // Keep the latest processed grid
          std::lock_guard<std::mutex> lock(grid_mutex_);
          latest_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(grid);
        }
      });
    cleaned_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "mapping/map_filtered", rclcpp::SensorDataQoS());
    grid_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
      "mapping/map_2d", rclcpp::QoS(1).reliable().transient_local());
    save_service_ = create_service<std_srvs::srv::Trigger>(
      "mapping/save_artifacts",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        save_artifacts(response);
      });
    timer_ = create_wall_timer(std::chrono::seconds(5), [this]() {
      RCLCPP_INFO(get_logger(),
        "Map snapshots=%llu; latest points=%llu -> %llu (SOR=%s, radius=%s)",
        static_cast<unsigned long long>(received_),
        static_cast<unsigned long long>(last_input_points_),
        static_cast<unsigned long long>(last_output_points_),
        sor_enabled_ ? "on" : "off", radius_enabled_ ? "on" : "off");
    });
  }

private:
  void save_artifacts(const std::shared_ptr<std_srvs::srv::Trigger::Response> & response)
  {
    if (output_directory_.empty()) {
      response->success = false;
      response->message = "output_directory is empty";
      return;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr grid;
    {
      std::lock_guard<std::mutex> lock(grid_mutex_);
      grid = latest_grid_;
    }
    if (!grid || grid->info.width == 0 || grid->info.height == 0 ||
      grid->data.size() != static_cast<std::size_t>(grid->info.width) * grid->info.height)
    {
      response->success = false;
      response->message = "No valid processed 2D map has been received";
      return;
    }

    const std::filesystem::path directory(output_directory_);
    std::error_code directory_error;
    if (!std::filesystem::is_directory(directory, directory_error)) {
      response->success = false;
      response->message = directory_error ?
        "Cannot access output directory: " + directory_error.message() :
        "Output directory does not exist: " + directory.string();
      return;
    }

    const auto pgm_path = directory / "map.pgm";
    const auto yaml_path = directory / "map.yaml";
    try {
      std::ofstream pgm(pgm_path, std::ios::binary | std::ios::trunc);
      if (!pgm) {
        response->success = false;
        response->message = "Cannot open " + pgm_path.string() + " for writing";
        return;
      }
      // Row 0 of a PGM is the top of the image, which is the highest map y.
      pgm << "P5\n" << grid->info.width << ' ' << grid->info.height << "\n255\n";
      std::vector<unsigned char> row(grid->info.width);
      for (std::uint32_t image_row = 0; image_row < grid->info.height; ++image_row) {
        const auto map_row = grid->info.height - 1 - image_row;
        for (std::uint32_t column = 0; column < grid->info.width; ++column) {
          const auto value = grid->data[
            static_cast<std::size_t>(map_row) * grid->info.width + column];
          row[column] = value < 0 ?
            205 :  // Unknown, same shading as nav2 map_saver.当未知的时候
            static_cast<unsigned char>(std::lround((100 - value) * 2.55));
        }
        pgm.write(
          reinterpret_cast<const char *>(row.data()), static_cast<std::streamsize>(row.size()));
      }
      pgm.close();
      if (!pgm) {
        response->success = false;
        response->message = "Failed while writing " + pgm_path.string();
        return;
      }

      const auto & origin = grid->info.origin;
      const auto & q = origin.orientation;//四元数
      const double yaw = std::atan2(//偏航角
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));

      std::ofstream yaml(yaml_path, std::ios::trunc);
      if (!yaml) {
        response->success = false;
        response->message = "Cannot open " + yaml_path.string() + " for writing";
        return;
      }
      yaml << std::setprecision(std::numeric_limits<double>::max_digits10);
      yaml << "image: map.pgm\n";
      yaml << "resolution: " << grid->info.resolution << "\n";
      yaml << "origin: [" << origin.position.x << ", " << origin.position.y << ", " << yaw <<
        "]\n";
      yaml << "negate: 0\n";
      yaml << "occupied_thresh: 0.65\n";
      yaml << "free_thresh: 0.196\n";
      yaml.close();
      if (!yaml) {
        response->success = false;
        response->message = "Failed while writing " + yaml_path.string();
        return;
      }

      response->success = true;
      response->message = "Saved " + pgm_path.string() + " and " + yaml_path.string();
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("Failed to save map artifacts: ") + error.what();
    }
  }

  std::uint64_t received_{0};
  std::uint64_t last_input_points_{0};
  std::uint64_t last_output_points_{0};
  std::string output_directory_;
  std::mutex grid_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_grid_;
  double voxel_size_{0.10};
  double grid_resolution_{0.10};
  double obstacle_height_min_{0.15};
  double obstacle_height_max_{1.50};
  bool sor_enabled_{false};
  int sor_mean_k_{0};
  double sor_stddev_multiplier_{0.0};
  bool radius_enabled_{false};
  double radius_search_{0.0};
  int radius_min_neighbors_{0};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cleaned_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {rclcpp::spin(std::make_shared<MapProcessorNode>());}
  catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("map_processor_node"), "%s", error.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
