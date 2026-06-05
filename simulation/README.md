# simulation

`simulation` 提供一个可配置的阿克曼小车仿真节点。它订阅 `/cmd_vel`，按阿克曼运动学积分车体位姿，并发布：

- `nav_msgs/msg/Odometry`，默认话题 `/fastlio2/lio_odom`
- `map -> base_link` TF
- 历史轨迹 `nav_msgs/msg/Path`，默认 `/simulation/path`
- 车体可视化 `visualization_msgs/msg/MarkerArray`，默认 `/simulation/vehicle_markers`

这样可以直接喂给 `lio_scan_monitor` 的 odom 输入，也可以在 RViz2 里观察小车运动。

## 话题

- 订阅 `/cmd_vel`，类型 `geometry_msgs/msg/Twist`
- 发布 `/fastlio2/lio_odom`，类型 `nav_msgs/msg/Odometry`
- 发布 `/simulation/path`，类型 `nav_msgs/msg/Path`
- 发布 `/simulation/vehicle_markers`，类型 `visualization_msgs/msg/MarkerArray`
- 广播 TF：`map -> base_link`

## 运动模型

- 使用自行车模型近似阿克曼小车
- `linear.x` 作为目标车速
- `angular.z` 会换算成目标前轮转角：`delta = atan(wheelbase * wz / vx)`
- 当 `cmd_vel` 超时未更新时，节点会自动减速并把转角回正

注意：

- 当 `linear.x` 接近 `0` 时，阿克曼车不能像差速车那样原地旋转，所以此时不会根据 `angular.z` 产生原地转向运动。
- 默认状态里，里程计和 TF 的 `base_link` 位姿对应的是参数 `rear_axle_to_base_link_x_m` 指定后的车体参考点。

## 参数

配置文件：`config/ackermann_simulator_params.yaml`

关键参数：

- `cmd_vel_topic`：控制输入，默认 `/cmd_vel`
- `odom_topic`：里程计输出，默认 `/fastlio2/lio_odom`
- `map_frame_id`：世界坐标系，默认 `map`
- `base_frame_id`：车体坐标系，默认 `base_link`
- `publish_rate_hz`：仿真与发布频率
- `cmd_timeout_sec`：控制超时后自动停车的时间
- `wheelbase_m`：轴距
- `track_width_m`：轮距
- `rear_axle_to_front_m` / `rear_axle_to_back_m`：后轴到前后车身边界的尺寸
- `rear_axle_to_base_link_x_m`：`base_link` 相对后轴中心的前向偏移
- `max_speed_mps`：速度限幅
- `max_steering_angle_deg`：最大前轮转角
- `max_acceleration_mps2` / `max_deceleration_mps2`：加减速限制
- `max_steering_rate_degps`：前轮转角变化速率限制
- `initial_x_m` / `initial_y_m` / `initial_yaw_deg`：初始位姿

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select simulation
source install/setup.bash
```

## 运行

```bash
ros2 launch simulation ackermann_simulator.launch.py
```

如果希望同时拉起仿真节点和 RViz2，并加载
`src/LidarBridge/projectConfigs/simulationConifg.rviz`：

```bash
ros2 launch simulation simulation_with_rviz.launch.py
```

给一个前进带转弯的控制：

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.8}, angular: {z: 0.4}}" -r 20
```

## RViz2 观察

1. `Fixed Frame` 设为 `map`
2. 添加 `TF`
3. 添加 `Path`，选择 `/simulation/path`
4. 添加 `MarkerArray`，选择 `/simulation/vehicle_markers`
5. 如果你同时启动 `lio_scan_monitor`，它的 `odom_topic` 可以直接保持默认 `/fastlio2/lio_odom`
