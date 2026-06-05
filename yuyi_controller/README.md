# yuyi_controller

`yuyi_controller` 是一个基于 Pure Pursuit 的路径跟踪控制包。它负责：

- 加载 `simulation/path_generator` 生成的路径文件
- 订阅 odom，读取小车当前位置和朝向
- 根据路径执行 Pure Pursuit 跟踪
- 发布 `/cmd_vel` 给仿真阿克曼小车

## 输入

- `odom_topic`，默认 `/fastlio2/lio_odom`
- `path_file`，支持：
  - `generated_path.yaml`
  - `generated_path.csv`

默认配置文件：

```text
config/yuyi_controller_params.yaml
```

默认 launch 会把路径文件指向：

```text
src/LidarBridge/simulation/paths/generated_path.yaml
```

## 输出

- `/cmd_vel`，类型 `geometry_msgs/msg/Twist`
- `/yuyi_controller/reference_path`，类型 `nav_msgs/msg/Path`
- `/yuyi_controller/lookahead_target`，类型 `visualization_msgs/msg/Marker`

## 关键参数

- `max_speed_mps`：最大线速度
- `max_acceleration_mps2`：最大加速度
- `max_deceleration_mps2`：最大减速度
- `lookahead_time_sec`：速度相关 lookahead 时间
- `min_lookahead_distance_m`：最小 lookahead
- `max_lookahead_distance_m`：最大 lookahead
- `goal_tolerance_m`：终点停止容差

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
