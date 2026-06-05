# LidarBridge 使用整理

`src/LidarBridge` 是上位机 LiDAR/FAST-LIO 感知和推料机下位机之间的桥接目录。它的目标是：用 `maps/poses.txt` 作为线路输入，用 `/fastlio2/lio_odom` 定位当前路径段，用 `/scan` 估计左右护栏距离，然后把当前路径段、目标延边距离、横向偏移和转弯信息整理成下位机可用的数据。

当前重点使用 `pusher_nav_bridge`。它现在是 dry-run 版本：会生成并打印 26 字节 payload Hex，但不会真实写 RS485。

更完整的算法细节见 [ALGORITHM.md](ALGORITHM.md)，包内简要说明见 [pusher_nav_bridge/README.md](pusher_nav_bridge/README.md)。

## 目录内包说明

| 包 | 作用 | 当前用途 |
|----|------|----------|
| `lio_scan_monitor` | 订阅 FAST-LIO odom 和 `/scan`，输出八方向最近障碍物，并可发布 RViz marker | 调试 LiDAR 点是否在车体坐标中正确、确认左右方向是否符合现场 |
| `simulation` | 订阅 `/cmd_vel`，模拟阿克曼小车运动，发布 `/fastlio2/lio_odom`、`map -> base_link` TF、轨迹和车体 marker | 本地联调 `lio_scan_monitor`、RViz2 和导航链路时的仿真输入 |
| `rs485_tester` | 测试 RS485 串口打开、串口参数和基础字节发送 | 只验证串口硬件，不是推料机导航业务协议 |
| `pusher_nav_bridge` | 编译 `poses.txt` 路线、估计侧向护栏距离、生成推料机 payload | 当前导航桥接主包，dry-run 打印 payload |

## 当前数据流

```text
maps/poses.txt
  -> 路线清洗、反向处理、重采样、分段、转角计算

/fastlio2/lio_odom
  -> 取 x/y，投影到当前路线段，得到 segment 和 s

/scan
  -> 侧向 ROI 点集
  -> RANSAC 拟合护栏线
  -> 当前侧距、横向偏移、置信度

固定任务参数
  -> 左/右目标延边距离、速度、超声波兼容字段

最终输出
  -> 26 字节 payload Hex 日志
```

注意：八方向最近障碍物适合观察调试，不建议直接作为导航闭环输入。真正用于延边导航时，应使用 `pusher_nav_bridge` 的侧向 ROI 线拟合结果。

## 输入

### 1. 路线文件

默认路线文件：

```text
maps/poses.txt
```

默认参数中写的是相对路径：

```yaml
poses_file: "maps/poses.txt"
```

因此建议从工作空间根目录运行节点：

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
```

当前解析逻辑支持这些格式，并只使用 `x/y` 做路线几何：

```text
frame.pcd x y z qw qx qy qz
x y z qw qx qy qz
prefix frame.pcd x y z qw qx qy qz
```

仓库中实际格式示例：

```text
12.pcd 6.11033 0.190023 0.0145 0.999988 -0.00038168 -0.00322504 -0.00354295
```

### 2. ROS 输入话题

| 话题参数 | 默认话题 | 类型 | 用途 |
|----------|----------|------|------|
| `odom_topic` | `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | 取当前机器人 `x/y`，判断当前处于哪一段路线 |
| `scan_topic` | `/scan` | `sensor_msgs/msg/LaserScan` | 估计当前左/右侧护栏距离 |

当前 dry-run 节点没有接 TF，`/scan` 点默认按 `base_link` 或等效车体坐标解释：

- `x` 为车体前方
- `y` 为车体左侧
- 左侧延边看 `y > 0` 的 ROI
- 右侧延边看 `y < 0` 的 ROI

### 3. 任务参数

配置文件：

```text
src/LidarBridge/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

关键参数：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `follow_side` | `left` | 当前跟随哪一侧护栏，取值 `left` 或 `right` |
| `travel_direction` | `forward` | 沿 `poses.txt` 正向还是反向行驶，取值 `forward` 或 `reverse` |
| `poses_reference_type` | `vehicle_center` | `poses.txt` 是车体中心线、左护栏线还是右护栏线 |
| `target_left_distance_m` | `1.10` | 目标左侧延边距离，单位 m |
| `target_right_distance_m` | `1.10` | 目标右侧延边距离，单位 m |
| `travel_speed_mmps` | `300` | 下发 payload 中的行走速度，单位 mm/s |
| `rotation_speed` | `200` | 下发 payload 中的旋转速度 |
| `ultrasonic_work_distance_mm` | `900` | 超声波兼容字段固定值 |
| `ultrasonic_adjust_distance_mm` | `100` | 超声波兼容字段固定值或调节阈值 |

`target_left_distance_m` 和 `target_right_distance_m` 是目标延边距离，不是 LiDAR 实测值，也不是里程。真实超声波当前不参与导航。

### 4. 侧距估计参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `side_roi_x_min_m` | `-2.0` | 侧向 ROI 前后范围最小 x |
| `side_roi_x_max_m` | `3.0` | 侧向 ROI 前后范围最大 x |
| `side_roi_y_min_m` | `0.2` | 离车体最近侧向距离，小于该值的点忽略 |
| `side_roi_y_max_m` | `5.0` | 最大侧向距离，超过该值的点忽略 |
| `side_distance_gate_m` | `1.0` | 实测侧距相对目标距离允许的筛选范围 |
| `ransac_inlier_threshold_m` | `0.08` | RANSAC 线拟合内点阈值 |
| `min_side_inliers` | `8` | 有效侧线最少内点数 |
| `line_heading_gate_deg` | `45.0` | 护栏线方向相对车体前进方向的最大允许偏差 |

现场调试时，如果 `side_detected=false` 长时间出现，优先检查 `/scan` 坐标、ROI 范围和目标延边距离。

## 输出

### 1. dry-run 日志

`pusher_nav_bridge_node` 每 500 ms 节流打印一次状态，格式类似：

```text
segment=1 s=2.50 side_detected=true dist=1.300 offset=0.200 confidence=0.82 payload=0x02 0x01 ...
```

字段含义：

| 字段 | 含义 |
|------|------|
| `segment` | 当前路线段编号，从 1 开始 |
| `s` | 当前机器人在路线上的累计弧长，单位 m |
| `side_detected` | 是否成功从 `/scan` 拟合出当前跟随侧护栏线 |
| `dist` | 当前跟随侧实测护栏距离，单位 m |
| `offset` | 当前跟随侧横向偏移，单位 m |
| `confidence` | 侧线拟合置信度，范围约 `0~1` |
| `payload` | 26 字节业务 payload 的 Hex 字符串 |

如果 `side_detected=false`，payload 中的 `横向距离` 会置 0，避免把无效侧距继续下发。

### 2. 26 字节 payload

当前 payload 只是不带帧头、CRC、ACK 的业务字段区。多字节字段当前按大端序编码。

| 字节范围 | 字段 | 类型 | 单位/含义 |
|----------|------|------|-----------|
| `0` | 路径条数 | `uint8` | 编译后的路线段总数 |
| `1` | 路径编号 | `uint8` | 当前路线段编号 |
| `2..5` | 左侧距离 | `uint32` | 目标左侧延边距离，单位 mm |
| `6..7` | 横向距离 | `int16` | 当前跟随侧横向偏移，单位 mm |
| `8..11` | 右侧距离 | `uint32` | 目标右侧延边距离，单位 mm |
| `12..13` | 行走速度 | `uint16` | 单位 mm/s |
| `14..15` | 超声波工作距离 | `uint16` | 兼容固定值，单位 mm |
| `16..17` | 超声波调节距离 | `uint16` | 兼容固定值，单位 mm |
| `18` | 横向拐弯角度 | `uint8` | 当前段末端转角绝对值，单位 degree |
| `19` | 旋转类型 | `uint8` | `0` 不转，`1` 左转，`2` 右转，`3` 掉头，`4` 原地转 |
| `20..21` | 旋转速度 | `uint16` | 来自配置 |
| `22..25` | 预留字段 | `uint32` | 当前置 0 |

真实接 RS485 前还需要和下位机确认完整帧格式，包括帧头、命令字、长度、CRC、ACK、重发策略、端序以及 `横向距离` 是否支持 `int16` 补码。

### 3. 横向偏移符号

当前统一定义：

```text
左侧延边: offset = +(left_measured_distance - target_left_distance)
右侧延边: offset = -(right_measured_distance - target_right_distance)
```

含义：

- 左侧延边时，`offset > 0` 表示车离左护栏太远。
- 右侧延边时，`offset < 0` 表示车离右护栏太远。

如果下位机纠偏方向定义相反，应只在协议适配层取反，不要改路线和侧距算法本身。

## 编译

在工作空间根目录执行：

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
colcon build --packages-select pusher_nav_bridge
source install/setup.bash
```

如果也要调试八方向 LiDAR 或 RS485 串口，可以一起编译：

```bash
colcon build --packages-select lio_scan_monitor rs485_tester pusher_nav_bridge
source install/setup.bash
```

## 测试

`pusher_nav_bridge` 当前有路线处理、侧距估计、协议编码和完整链路测试：

```bash
ctest --test-dir build/pusher_nav_bridge --output-on-failure
colcon test --packages-select pusher_nav_bridge --event-handlers console_direct+
colcon test-result --test-result-base build/pusher_nav_bridge --verbose
```

文档修改不需要重新跑测试；代码、参数默认值或协议字段变更后建议跑完整测试。

## 运行

### 1. 运行前检查

先确认路线文件存在：

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
ls maps/poses.txt
```

再确认上位机已经发布 odom 和 scan：

```bash
ros2 topic list | grep -E '/fastlio2/lio_odom|/scan'
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /scan
```

如果话题名不同，修改 `odom_topic` 和 `scan_topic`。

### 2. 使用 launch 运行 dry-run 节点

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
source install/setup.bash
ros2 launch pusher_nav_bridge pusher_nav_bridge.launch.py
```

launch 默认读取安装后的配置：

```text
install/pusher_nav_bridge/share/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

如果修改了源码目录下的配置文件，建议重新 build，或用下面的 `ros2 run --params-file` 方式直接指定源码配置。

### 3. 直接指定源码配置运行

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
source install/setup.bash
ros2 run pusher_nav_bridge pusher_nav_bridge_node \
  --ros-args \
  --params-file src/LidarBridge/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

临时覆盖参数示例：

```bash
ros2 run pusher_nav_bridge pusher_nav_bridge_node \
  --ros-args \
  --params-file src/LidarBridge/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml \
  -p poses_file:=$(pwd)/maps/poses.txt \
  -p follow_side:=left \
  -p target_left_distance_m:=1.10 \
  -p target_right_distance_m:=1.10
```

### 4. 调试八方向 LiDAR

用于确认 `/scan`、TF 和左右方向是否正常：

```bash
source install/setup.bash
ros2 launch lio_scan_monitor lio_scan_monitor.launch.py
```

RViz2 中添加 `MarkerArray`，选择：

```text
/lio_scan_monitor/obstacles
```

### 5. 测试 RS485 串口

只用于验证串口硬件和基础发送，不代表推料机导航 payload 已经接入：

```bash
source install/setup.bash
ros2 launch rs485_tester rs485_test.launch.py
```

串口参数在：

```text
src/LidarBridge/rs485_tester/config/rs485_params.yaml
```

## 如何判断运行正常

启动后首先应该看到：

```text
Loaded <route_points> route points and <segments> segments from maps/poses.txt; dry-run protocol is enabled
```

如果还没有收到 odom，会看到：

```text
waiting for odometry
```

当 odom 和 scan 都正常时，应持续看到：

```text
segment=... s=... side_detected=... dist=... offset=... confidence=... payload=...
```

判断顺序：

1. `segment` 会随车辆沿 `poses.txt` 移动而变化。
2. `s` 应该随前进方向递增。
3. `side_detected=true` 时，`dist` 应接近现场车体到护栏的实际距离。
4. `offset` 应等于实测距离减目标距离，并按左/右侧符号规则输出。
5. `payload` 应每行都有 26 个 Hex 字节。

## 常见问题

### `fatal error: cannot open poses file: maps/poses.txt`

原因通常是运行目录不在工作空间根目录，或者参数文件中 `poses_file` 路径不正确。

处理方式：

```bash
cd /media/bigdisk/dog_ws_ros/src/yuyi_pusher_bot
ros2 launch pusher_nav_bridge pusher_nav_bridge.launch.py
```

或者把 `poses_file` 改为绝对路径。

### 一直 `waiting for odometry`

说明节点没有收到 `odom_topic`。检查：

```bash
ros2 topic list
ros2 topic echo /fastlio2/lio_odom --once
```

如果 FAST-LIO 话题名不同，修改 `odom_topic`。

### `side_detected=false`

常见原因：

- `/scan` 不是车体坐标，当前节点没有做 TF 转换。
- `follow_side` 配错，例如现场护栏在左侧但配置为 `right`。
- ROI 范围不覆盖护栏。
- `target_left_distance_m` 或 `target_right_distance_m` 与现场差距太大，被 `side_distance_gate_m` 过滤。
- 护栏点太少，未达到 `min_side_inliers`。

### payload 有输出但下位机不能直接用

当前输出只是业务字段区 Hex，不包含真实 RS485 完整帧。接下位机前必须补齐或确认：

- 帧头和命令字
- 长度字段
- CRC 或校验和
- ACK 与重发策略
- 多字节端序
- `横向距离` 正负方向
- 是否仍要求 Modbus RTU 格式

## 当前限制

- 不依赖 Navigation2，也不订阅规划路径。
- dry-run 节点不写真实 RS485。
- dry-run 节点未接 TF，默认 `/scan` 已经在 `base_link` 或等效车体坐标。
- `poses.txt` 当前主要使用 `x/y`，yaw/quaternion 暂只保留为输入兼容，不直接作为路径段方向。
- 当前只输出当前跟随侧的一个 `横向距离` 字段；如果下位机需要同时接收左右实测距离，需要扩展协议或使用预留字段。
- 超声波字段当前是兼容固定值，不读取真实超声波。

## 建议现场使用顺序

1. 先运行 `lio_scan_monitor`，确认 `/scan` 左右方向和障碍物距离符合现场。
2. 确认 `maps/poses.txt` 是车体中心线、左护栏线还是右护栏线，并配置 `poses_reference_type`。
3. 配置 `follow_side`、`travel_direction`、目标左右延边距离。
4. 编译并运行 `pusher_nav_bridge` dry-run。
5. 对照现场实测距离检查 `dist`、`offset` 和 `payload`。
6. 和下位机确认完整 RS485 帧格式后，再把 dry-run payload 接入真实串口发送。
