# lio_scan_monitor

`lio_scan_monitor` 是 LiDAR/FAST-LIO 感知调试包，用于观察定位位姿和八方向最近障碍物。它不负责 RS485 下发，也不应直接承载推料机业务协议。

## 话题

- 订阅 `/fastlio2/lio_odom`，类型为 `nav_msgs/msg/Odometry`
- 订阅 `/scan`，类型为 `sensor_msgs/msg/LaserScan`
- 发布 `/lio_scan_monitor/obstacles`，类型为 `visualization_msgs/msg/MarkerArray`

## 功能

- 从 odometry quaternion 中计算并打印 `x`、`y`、`z` 和 yaw。
- 在终端中刷新同一个状态面板，避免重复刷屏。
- 把 `/scan` 划分为八个方向：`front`、`front_left`、`left`、`back_left`、`back`、`back_right`、`right`、`front_right`。
- 通过 TF 把有效 scan 点转换到 `base_link`，然后输出每个方向最近点的距离、`x/y` 坐标和角度。
- 可选发布 RViz2 标记，包括方向扇区、方向标签、最近障碍点、箭头和距离文本。

## 和推料机延边导航的关系

这个包适合作为调试入口，确认 LiDAR 点是否正确落在 `base_link` 中，以及八方向点是否符合现场直觉。真正用于延边导航时，不建议直接使用打印出来的 `left.distance_m` 或 `right.distance_m`。

原因：

- `left.distance_m` 是左侧扇区内最近点的径向距离，不一定是护栏到车体的横向距离。
- 单个最近点不能估计车体相对护栏的姿态角。
- 现场护栏是连续线体，应优先用侧向 ROI 点集拟合护栏线。

推荐策略：

- 左侧延边：优先拟合左侧 ROI 点集；早期兜底可用 `front_left + back_left` 两点估计左侧护栏线。
- 右侧延边：优先拟合右侧 ROI 点集；早期兜底可用 `front_right + back_right` 两点估计右侧护栏线。
- `left.y_m` 和 `right.y_m` 只能作为 90 度方向测距校验，不应单独作为导航闭环输入。

## 编译

在 ROS 2 工作空间根目录执行：

```bash
colcon build --packages-select lio_scan_monitor
source install/setup.bash
```

## 运行

```bash
ros2 launch lio_scan_monitor lio_scan_monitor.launch.py
```

在 RViz2 中添加 `MarkerArray` 显示，选择 `/lio_scan_monitor/obstacles`。

## 配置

配置文件：`config/lio_scan_monitor_params.yaml`

- `odom_topic`：里程计话题，默认 `/fastlio2/lio_odom`
- `scan_topic`：激光扫描话题，默认 `/scan`
- `base_frame_id`：用于障碍物距离和方向判断的车体坐标系，默认 `base_link`
- `scan_sector_width_deg`：每个方向扇区角宽，默认 `45.0`
- `log_period_ms`：终端刷新周期，默认 `500`
- `terminal_use_color`：终端是否使用 ANSI 颜色，默认 `true`
- `terminal_refresh_in_place`：是否原地刷新终端面板，默认 `true`
- `enable_visualization`：是否发布 RViz2 marker，默认 `true`
- `enable_sector_visualization`：是否发布八方向扇区和标签，默认 `true`
- `visualization_topic`：marker 话题，默认 `/lio_scan_monitor/obstacles`
- `marker_lifetime_sec`：marker 生命周期，默认 `1.0`
- `sector_marker_radius_m`：扇区可视化半径，默认 `3.0`
- `sector_fill_alpha`：扇区填充透明度，默认 `0.08`
