# RoboMaster assignment 5 Mapping

使用录好的 MID-360 雷达和 IMU 数据（rosbag），通过 FAST-LIO 建立三维点云地图并输出轨迹，再由独立的处理节点完成离群点过滤、体素降采样和二维栅格投影，最终保存三维 PCD、二维 PGM 与 YAML。具体见 [作业要求](docs/assignment.md)。

开发环境为 **Ubuntu 22.04 / ROS 2 Humble**。

## 完成内容

- `mapping_node`
  - 订阅 bag 中的 `livox_ros_driver2/msg/CustomMsg` 与 `sensor_msgs/msg/Imu`，转发到 FAST-LIO 使用的 `/livox/lidar`、`/livox/imu`
  - 用开头若干帧 IMU 估计重力方向，把 FAST-LIO 输出的地图、里程计和轨迹旋转到重力对齐的 `map` 坐标系，并把地面平移到 `z=0`
  - 发布 `map -> camera_init` 静态 TF，订阅 FAST-LIO 的 `/Laser_map`、`/Odometry`、`/path`，转发到 `/mapping/map`、`/mapping/odom`、`/mapping/path`
  - 提供 `mapping/save_map` 服务，把最近一帧三维地图保存为 PCD
- `map_processor_node`
  - 可选 SOR / 半径离群点过滤，体素降采样减少点数
  - 保留高度带内的点，投影为 `nav_msgs/msg/OccupancyGrid`，发布到 `/mapping/map_2d`
  - 提供 `mapping/save_artifacts` 服务，把最新二维地图保存为 PGM 与 YAML
- `rviz/mapping.rviz`
  - 同时显示三维地图、轨迹和二维栅格，`Fixed Frame` 使用重力对齐后的 `map`

## 数据

bag 目录包含 `metadata.yaml` 和 `.db3` 文件，回放时通过 `bag_path` 或命令行指定，不需要提交到仓库。

数据时长约 104 秒。点云记录周围物体的位置，IMU 记录角速度和加速度

| 话题 | 类型 |
|---|---|
| `/livox/lidar_192_168_1_140` | `livox_ros_driver2/msg/CustomMsg` |
| `/livox/imu_192_168_1_140` | `sensor_msgs/msg/Imu` |

## 依赖

### 1. Livox-SDK2

按 [SDK 官方说明](https://github.com/Livox-SDK/Livox-SDK2#2-installation) 编译安装。它是下一步编译完整 Livox 驱动包所需的底层库。

### 2. livox_ros_driver2

这份 bag 使用 `livox_ros_driver2/msg/CustomMsg`。安装这个包后，ROS 才能识别并回放该消息

使用独立的 `~/ws_livox` 工作空间，因为官方构建脚本会清理所在工作空间的构建产物。下面先准备 ROS 2 的依赖清单，再安装依赖、执行 [Humble 构建脚本](https://github.com/Livox-SDK/livox_ros_driver2#23-build-the-livox-ros-driver-2)。

```bash
mkdir -p ~/ws_livox/src
cd ~/ws_livox/src
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
cd livox_ros_driver2
cp package_ROS2.xml package.xml
source /opt/ros/humble/setup.bash
rosdep install --from-paths . --ignore-src -r -y --rosdistro humble
./build.sh humble
source ~/ws_livox/install/setup.bash

# 能显示消息字段，说明 ROS 已找到 CustomMsg
ros2 interface show livox_ros_driver2/msg/CustomMsg
```

### 3. FAST-LIO

采用 [FAST-LIO](https://github.com/hku-mars/FAST_LIO) 的 ROS 2 分支：

```bash
cd ~/ws_livox/src
git clone -b ROS2 https://github.com/hku-mars/FAST_LIO.git
cd ~/ws_livox
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select fast_lio
source install/setup.bash
```

本仓库的 `mid360_mapping` 还需要 PCL、`pcl_conversions`、`tf2` 和 `tf2_ros`，相关依赖已写入 `package.xml` 和 `CMakeLists.txt`。

每次打开新终端，在本仓库根目录依次加载以下环境，再启动节点或回放数据：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
```

使用 zsh 时将上述 `setup.bash` 换成 `setup.zsh`

## 项目结构

| 路径 | 说明 |
|---|---|
| `src/mid360_mapping/src/mapping_node.cpp` | Livox 数据转发、FAST-LIO 输出转发与 PCD 保存 |
| `src/mid360_mapping/src/map_processor_node.cpp` | 点云过滤、降采样、二维栅格生成与保存 |
| `src/mid360_mapping/config/mid360_recorded.yaml` | 录制 bag 的输入话题、FAST-LIO 中转话题与 PCD 路径 |
| `src/mid360_mapping/config/map_processing.yaml` | 地图处理参数与二维地图输出目录 |
| `src/mid360_mapping/launch/mapping.launch.py` | 启动两个节点和 rviz |
| `src/mid360_mapping/launch/replay.launch.py` | 启动节点并回放 bag |
| `src/mid360_mapping/rviz/mapping.rviz` | 三维地图与二维地图显示配置 |

## 如何编译

在仓库根目录：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
colcon build --packages-select mid360_mapping
source install/setup.bash
```

## 运行方式

先启动 FAST-LIO（终端 A）：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=mid360.yaml rviz:=false use_sim_time:=true
```

再启动本仓库的节点和 rviz（终端 B）：

```bash
cd <仓库根目录>
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
ros2 launch mid360_mapping mapping.launch.py rviz:=true
```

最后回放 bag（终端 C）：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_livox/install/setup.bash
source install/setup.bash
ros2 bag play <bag 目录> --clock 60 --rate 1.0 \
  --topics /livox/lidar_192_168_1_140 /livox/imu_192_168_1_140
```

也可以用一个 launch 同时启动节点并回放：

```bash
ros2 launch mid360_mapping replay.launch.py \
  bag_path:=<bag 目录> rate:=1.0 start_paused:=false rviz:=true
```

`replay.launch.py` 只回放雷达和 IMU 话题，并使用 `--clock` 提供仿真时间，FAST-LIO 仍需先启动。

启动后 `mapping_node` 会先收集 `gravity_sample_count` 帧 IMU 估计重力方向，这段时间暂不发布 `/mapping/*`；bag 开头应保持静止。

## 如何保存地图

节点运行期间调用服务，保存当前缓存的最新一帧地图：

```bash
# 三维地图 -> maps/fastlio_map.pcd
ros2 service call /mapping/save_map std_srvs/srv/Trigger

# 二维地图 -> maps/map.pgm + maps/map.yaml
ros2 service call /mapping/save_artifacts std_srvs/srv/Trigger
```

输出路径由配置中的相对路径决定：

- `mid360_recorded.yaml`：`map_output_path: maps/fastlio_map.pcd`
- `map_processing.yaml`：`output_directory: maps`

因此需要从仓库根目录启动节点，产物统一写入 `maps/`。服务保存的是最近一次收到的地图，调用时节点需要仍在运行。

## 节点参数

两个节点都在启动时读取参数，修改后需要重启节点；`use_sim_time` 是 ROS 2 公共参数，当前两份配置都设为 `true`，与 `ros2 bag play --clock` 配合使用。

### `mapping_node`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_type` | `pointcloud2` | 输入类型，可选 `pointcloud2` 或 `livox_custom`；当前配置为 `livox_custom` |
| `lidar_topic` | `/livox/lidar` | 原始点云输入；当前配置为 `/livox/lidar_192_168_1_140` |
| `imu_topic` | `/livox/imu` | 原始 IMU 输入；当前配置为 `/livox/imu_192_168_1_140` |
| `fastlio_lidar_topic` | `/livox/lidar` | 转发给 FAST-LIO 的点云话题 |
| `fastlio_imu_topic` | `/livox/imu` | 转发给 FAST-LIO 的 IMU 话题 |
| `fastlio_map_topic` | `/Laser_map` | FAST-LIO 三维地图话题 |
| `fastlio_odom_topic` | `/Odometry` | FAST-LIO 里程计话题 |
| `fastlio_path_topic` | `/path` | FAST-LIO 轨迹话题 |
| `fastlio_world_frame` | `camera_init` | FAST-LIO 输出所用的世界坐标系 |
| `gravity_align` | `true` | 是否把输出旋转到重力对齐的 `map` 系并发布静态 TF |
| `gravity_sample_count` | `200` | 估计重力使用的 IMU 帧数，建议 bag 开头保持静止 |
| `map_output_path` | `maps/fastlio_map.pcd` | `save_map` 保存的 PCD 路径，相对启动目录 |
| `map_frame` | `map` | 重力对齐后的输出坐标系 |
| `body_frame` | `body` | 里程计输出使用的 child frame，应与 FAST-LIO 的 body 一致 |

这些参数以只读方式声明，运行中不能修改；话题参数不能为空。`gravity_align` 为 `false` 时直接透传 FAST-LIO 输出，不再发布 `map -> camera_init` TF，此时 rviz 的 `Fixed Frame` 需要改回 `camera_init`。

### `map_processor_node`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `input_topic` | `/mapping/map` | 输入的三维点云话题 |
| `output_directory` | `maps` | `save_artifacts` 保存 PGM/YAML 的目录，相对启动目录 |
| `voxel_size` | `0.10` | 体素降采样叶子大小，单位 m，必须大于 0 |
| `grid_resolution` | `0.10` | 二维栅格分辨率，单位 m，必须大于 0 |
| `obstacle_height_min` | `0.15` | 相对地面的最低高度，单位 m；`map` 的 `z=0` 为地面 |
| `obstacle_height_max` | `1.50` | 相对地面的最高高度，单位 m，必须大于最小值 |
| `sor_enabled` | `false` | 是否启用统计离群点过滤 |
| `sor_mean_k` | `0` | SOR 邻域点数，启用时需不小于 2 |
| `sor_stddev_multiplier` | `0.0` | SOR 标准差倍数，启用时需不小于 0 |
| `radius_enabled` | `false` | 是否启用半径离群点过滤 |
| `radius_search` | `0.0` | 半径搜索距离，单位 m，启用时需大于 0 |
| `radius_min_neighbors` | `0` | 半径内最少邻居数，启用时需不小于 1 |
| `map_frame` | `map` | 已声明，当前实现未使用 |

修改 `map_processing.yaml` 后，可以直接把源文件作为参数启动，避免重新安装：

```bash
ros2 launch mid360_mapping mapping.launch.py \
  map_params_file:=<仓库根目录>/src/mid360_mapping/config/map_processing.yaml \
  rviz:=true
```

`mapping_node` 的配置也可以用同样方式覆盖：

```bash
ros2 launch mid360_mapping mapping.launch.py \
  params_file:=<仓库根目录>/src/mid360_mapping/config/mid360_recorded.yaml \
  rviz:=true
```

对比参数时，建议在相同 bag 和相同时间点下分别记录输出点数、二维地图尺寸和障碍物轮廓，再比较 `maps/` 中生成的结果。

## 显示结果

rviz 中同时显示：

- `/mapping/map`：FAST-LIO 输出的三维点云地图
- `/mapping/path`：运动轨迹
- `/mapping/map_2d`：二维 `OccupancyGrid`

FAST-LIO 的世界坐标系是 `camera_init`，它取初始 IMU 姿态，不保证与重力对齐。`mapping_node` 用 bag 开头的 IMU 估计重力方向，把三维点云、里程计和轨迹旋转到 `map` 系，并把地面高度平移到 `z=0`；同时发布 `map -> camera_init` 静态 TF。rviz 的 `Fixed Frame` 设为 `map`。

三维点云和二维栅格都在 `map` 系中，二维地图的 `origin` 取滤波后点云的最小 `x/y`，`origin.z = 0` 对应地面，`resolution` 为 `0.1 m`，所以两者的位置、方向、范围和格子大小一致。

截图：

<!-- 在此插入 rviz 截图：同时显示 /mapping/map 和 /mapping/map_2d -->