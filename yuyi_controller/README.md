# yuyi_controller

`yuyi_controller` 是一个基于 Pure Pursuit 的路径跟踪控制包。它负责：

- 加载 `simulation/path_generator` 生成的路径文件
- 读取 `map -> base_link` TF，获取小车当前位置和朝向
- 根据路径执行 Pure Pursuit 跟踪
- 发布 `/cmd_vel` 给仿真阿克曼小车
- 发布 `/yuyi_controller/cmd_vel_marker` 用于 RViz2 可视化命令速度矢量

## 输入

- `map_frame_id`，默认 `map`
- `base_frame_id`，默认 `base_link`
- `path_file`，支持：
  - `generated_path.yaml`
  - `generated_path.csv`

默认配置文件：

```text
config/yuyi_controller_params.yaml
```

默认配置会把路径文件指向：

```text
src/LidarBridge/simulation/paths/generated_path.yaml
```

默认 launch 不会覆盖这个配置；只有显式传入 `path_file:=...` 时才会覆盖。

## 输出

- `/cmd_vel`，类型 `geometry_msgs/msg/Twist`
- `/yuyi_controller/reference_path`，类型 `nav_msgs/msg/Path`
- `/yuyi_controller/lookahead_target`，类型 `visualization_msgs/msg/Marker`
- `/yuyi_controller/cmd_vel_marker`，类型 `visualization_msgs/msg/Marker`

## 关键参数

- `max_speed_mps`：最大线速度
- `max_acceleration_mps2`：最大加速度
- `max_deceleration_mps2`：最大减速度
- `lookahead_time_sec`：速度相关 lookahead 时间
- `min_lookahead_distance_m`：最小 lookahead
- `max_lookahead_distance_m`：最大 lookahead
- `goal_tolerance_m`：终点停止容差
- `cmd_vel_marker_topic`：命令速度矢量 marker 话题
- `show_cmd_vel_marker`：是否发布命令速度矢量 marker
- `cmd_vel_marker_scale`：命令速度矢量长度缩放系数

## Scan Brake

- `use_scan_brake`：是否启用 `/scan` 制动
- `scan_topic`：激光雷达话题，默认 `/scan`
- `scan_max_age_sec`：扫描超时时间，超过该时间没有新扫描就停车
- `scan_brake.front.enabled`：前方扇区是否参与制动
- `scan_brake.front.brake_distance_m`：前方扇区制动距离

其余 7 个方向参数名称分别为：

- `scan_brake.left_front.*`
- `scan_brake.left.*`
- `scan_brake.left_rear.*`
- `scan_brake.rear.*`
- `scan_brake.right_rear.*`
- `scan_brake.right.*`
- `scan_brake.right_front.*`

当 `use_scan_brake=true` 时，控制器会把 `/scan` 按车体坐标切成 8 个固定方向扇区。只要任一已启用方向的最近有效距离小于该方向的 `brake_distance_m`，控制器就会按 `max_deceleration_mps2` 逐步刹停；当所有已启用方向恢复安全后，再按 `max_acceleration_mps2` 恢复行走。

运行时调参示例：

```bash
ros2 param set /yuyi_controller_node use_scan_brake true
ros2 param set /yuyi_controller_node scan_brake.front.enabled true
ros2 param set /yuyi_controller_node scan_brake.front.brake_distance_m 0.8
ros2 param set /yuyi_controller_node scan_max_age_sec 0.3
```

## 编译

```bash
colcon build --packages-select yuyi_controller
source install/setup.bash
```

## 运行

```bash
ros2 launch yuyi_controller yuyi_controller.launch.py
```

如果要指定另一条路径：

```bash
ros2 launch yuyi_controller yuyi_controller.launch.py \
  path_file:=/abs/path/to/generated_path.yaml
```

如果你想一键拉起 `simulation + map + rviz2 + yuyi_controller`，可以直接运行：

```bash
ros2 launch simulation simulation_with_controller.launch.py
```
