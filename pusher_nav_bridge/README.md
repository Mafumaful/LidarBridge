# pusher_nav_bridge

`pusher_nav_bridge` 是一个 ROS 2 C++ 包，用于把 FAST-LIO 定位、二维激光雷达侧向测距和 `poses.txt` 路线文件整理成推料机延边导航下位机可以理解的业务 payload。

当前版本是 **dry-run 桥接节点**：

- 会读取路线文件并编译路径段。
- 会订阅里程计和激光雷达。
- 会估计左侧或右侧护栏距离。
- 会生成 26 字节业务 payload 并在日志中打印 Hex。
- **不会写真实 RS485 串口，也不包含帧头、CRC、ACK 或重发逻辑。**

## 适用场景

这个包适合在接入真实下位机协议之前完成以下工作：

- 验证 `maps/poses.txt` 路线是否能被稳定分段。
- 验证 FAST-LIO 当前 `x/y` 是否能定位到正确路线段。
- 验证 `/scan` 在车体坐标下能否拟合出左侧或右侧护栏线。
- 验证目标延边距离、横向偏移、速度和转弯信息能否被编码为固定长度 payload。
- 和下位机开发人员对齐 26 字节业务字段的含义和端序。

## 包结构

```text
pusher_nav_bridge/
  CMakeLists.txt
  package.xml
  config/
    pusher_nav_bridge_params.yaml
  launch/
    pusher_nav_bridge.launch.py
  include/pusher_nav_bridge/
    protocol.hpp
    route_processing.hpp
    side_distance_estimator.hpp
    types.hpp
  src/
    protocol.cpp
    route_processing.cpp
    side_distance_estimator.cpp
    pusher_nav_bridge_node.cpp
```

主要模块：

| 模块 | 文件 | 作用 |
|------|------|------|
| 路线处理 | `route_processing.*` | 读取 `poses.txt`，清洗点、反向、重采样、分段、计算转角，并把当前 odom 位置投影到路线段 |
| 侧距估计 | `side_distance_estimator.*` | 从 `/scan` 转换出的车体坐标点中筛选侧向 ROI，用线拟合估计当前跟随侧护栏距离 |
| payload 协议 | `protocol.*` | 把当前路径段、目标距离、横向偏移、速度和转弯信息编码为 26 字节业务 payload |
| ROS 节点 | `pusher_nav_bridge_node.cpp` | 声明参数、加载路线、订阅 odom 和 scan、打印 dry-run payload |

## 数据流

```text
maps/poses.txt
  -> load_pose_xy_file()
  -> compile_route()
  -> route points + route segments + turn metadata

/fastlio2/lio_odom
  -> latest x/y
  -> locate_on_route()
  -> current segment id + route s

/scan
  -> LaserScan polar ranges
  -> base-frame x/y points
  -> estimate_side_distance()
  -> measured side distance + lateral offset + confidence

route segment + side estimate + static task parameters
  -> build_frame_from_segment()
  -> encode_payload()
  -> payload_to_hex()
  -> throttled ROS log
```

## 输入话题

| 参数 | 默认话题 | 类型 | 用途 |
|------|----------|------|------|
| `odom_topic` | `/fastlio2/lio_odom` | `nav_msgs/msg/Odometry` | 读取 `pose.pose.position.x/y`，用于判断机器人位于哪一个路线段 |
| `scan_topic` | `/scan` | `sensor_msgs/msg/LaserScan` | 用于估计当前跟随侧的护栏距离 |

当前节点没有做 TF 转换。`/scan` 必须已经位于车体坐标系或等效坐标系中：

- `x > 0` 表示车体前方。
- `y > 0` 表示车体左侧。
- `y < 0` 表示车体右侧。

如果 `/scan` 仍在雷达自身坐标系，且雷达坐标轴和车体坐标轴不一致，应先在上游完成坐标转换，或在本包中增加 TF 支持后再用于实车闭环。

## 路线文件

默认路线文件参数：

```yaml
poses_file: "maps/poses.txt"
```

这是相对路径。节点启动时会按当前工作目录解析它。推荐从 ROS 工作空间根目录运行，或者把 `poses_file` 改成绝对路径。

### 支持格式

解析器会跳过空行、`#` 开头的注释行、无法解析的行和非有限数值点。当前只使用 `x/y`，不会使用 `z` 或 quaternion。

支持 7、8、9 列输入：

```text
x y z qw qx qy qz
frame.pcd x y z qw qx qy qz
prefix frame.pcd x y z qw qx qy qz
```

常见示例：

```text
12.pcd 6.11033 0.190023 0.0145 0.999988 -0.00038168 -0.00322504 -0.00354295
```

对于 8 列格式，节点会取第 2、3 列作为 `x/y`。对于 9 列格式，节点会取第 3、4 列作为 `x/y`。对于 7 列格式，节点会取第 1、2 列作为 `x/y`。

### 路线参考类型

`poses_reference_type` 用来说明 `poses.txt` 中的 `x/y` 点代表什么：

| 取值 | 含义 |
|------|------|
| `vehicle_center` | `poses.txt` 是车辆中心线 |
| `left_guardrail` | `poses.txt` 是左侧护栏线，节点会按 `target_left_distance_m` 向右偏移得到车辆中心线 |
| `right_guardrail` | `poses.txt` 是右侧护栏线，节点会按 `target_right_distance_m` 向左偏移得到车辆中心线 |

注意：左右方向是在应用 `travel_direction` 之后按行驶方向计算的。因此反向行驶时，应配置 `travel_direction: "reverse"`，不要手动交换左右目标距离。

## 路线处理逻辑

节点启动时只编译一次路线。流程如下：

1. 读取 `poses_file` 中的有效 `x/y` 点。
2. 删除间距小于 `min_point_gap_m` 的连续近邻点。
3. 如果 `travel_direction=reverse`，反转点序。
4. 按 `resample_step_m` 对中心线重采样。
5. 使用 `heading_window_m` 计算平滑航向。
6. 如果路线文件代表护栏线，根据 `poses_reference_type` 偏移得到车辆中心线。
7. 运行 RDP 折线简化，使用 `rdp_tolerance_m` 提取候选转角点。
8. 根据候选转角点构建路线段，并过滤短于 `min_segment_length_m` 的中间段。
9. 根据相邻路线段航向差计算转角和旋转类型。

旋转类型枚举：

| 值 | 名称 | 触发规则 |
|----|------|----------|
| `0` | `None` | 转角绝对值小于 `5 deg` |
| `1` | `Left` | 正转角 |
| `2` | `Right` | 负转角 |
| `3` | `UTurn` | 转角绝对值大于 `150 deg` |
| `4` | `InPlace` | 当前代码中保留枚举，路线分段暂不主动生成 |

在线运行时，节点会把最新 odom `x/y` 投影到每个路线段，选择距离最近的投影结果作为当前路线位置。日志中的 `segment` 和 `s` 来自这个定位结果。

## 侧向距离估计

`estimate_side_distance()` 从 LaserScan 转换出的车体坐标点中估计当前跟随侧护栏线。

处理流程：

1. 根据 `follow_side` 选择左侧点或右侧点。
2. 使用 `side_roi_x_min_m`、`side_roi_x_max_m` 限制前后范围。
3. 使用 `side_roi_y_min_m`、`side_roi_y_max_m` 限制侧向距离范围。
4. 优先使用 `side_distance_gate_m` 保留接近目标延边距离的点。
5. 如果过滤后点数不足 `min_side_inliers`，放宽目标距离 gate，再筛选一次 ROI。
6. 对候选点做穷举式 RANSAC 线拟合。
7. 要求内点数不少于 `min_side_inliers`。
8. 要求拟合线方向相对车体前进方向不超过 `line_heading_gate_deg`。
9. 输出实测侧距、横向偏移、航向误差、内点数和置信度。

横向偏移定义：

```text
左侧延边: offset = +(left_measured_distance - target_left_distance)
右侧延边: offset = -(right_measured_distance - target_right_distance)
```

含义：

- 左侧延边时，`offset > 0` 表示车辆离左侧护栏太远。
- 右侧延边时，`offset < 0` 表示车辆离右侧护栏太远。

如果下位机纠偏方向定义与这里相反，建议只在协议适配层取反，不要改路线处理或侧距估计逻辑。

## 参数说明

默认参数文件：

```text
pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

### 基础输入参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `poses_file` | `maps/poses.txt` | 路线文件路径，可以是相对路径或绝对路径 |
| `odom_topic` | `/fastlio2/lio_odom` | odom 输入话题 |
| `scan_topic` | `/scan` | LaserScan 输入话题 |
| `follow_side` | `left` | 当前跟随侧，支持 `left` 或 `right`。其他字符串会被当作 `left` |

### 路线参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `travel_direction` | `forward` | 行驶方向，支持 `forward` 或 `reverse`。其他字符串会被当作 `forward` |
| `poses_reference_type` | `vehicle_center` | 路线文件参考类型，支持 `vehicle_center`、`left_guardrail`、`right_guardrail` |
| `target_left_distance_m` | `1.10` | 目标左侧延边距离，单位 m |
| `target_right_distance_m` | `1.10` | 目标右侧延边距离，单位 m |
| `min_point_gap_m` | `0.05` | 清洗路线时保留连续点的最小间距，单位 m |
| `resample_step_m` | `0.50` | 路线重采样步长，单位 m，代码内部最小使用 `0.05 m` |
| `heading_window_m` | `0.50` | 计算平滑航向的窗口半宽，单位 m，代码内部最小使用 `0.05 m` |
| `rdp_tolerance_m` | `0.15` | RDP 折线简化容差，单位 m |
| `min_segment_length_m` | `0.50` | 路线段最小长度，单位 m |

### 侧距估计参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `side_roi_x_min_m` | `-2.0` | ROI 前后方向最小 `x`，单位 m |
| `side_roi_x_max_m` | `3.0` | ROI 前后方向最大 `x`，单位 m |
| `side_roi_y_min_m` | `0.2` | ROI 侧向最小距离，单位 m |
| `side_roi_y_max_m` | `5.0` | ROI 侧向最大距离，单位 m |
| `side_distance_gate_m` | `1.0` | 候选点相对目标距离的允许偏差，单位 m |
| `ransac_inlier_threshold_m` | `0.08` | 线拟合内点阈值，单位 m |
| `min_side_inliers` | `8` | 有效护栏线所需最少内点数 |
| `line_heading_gate_deg` | `45.0` | 拟合线方向相对车体前进方向的最大允许偏差，单位 degree |

### 下位机业务参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `travel_speed_mmps` | `300` | 行走速度，单位 mm/s，编码为 `uint16` |
| `rotation_speed` | `200` | 旋转速度，编码为 `uint16` |
| `ultrasonic_work_distance_mm` | `900` | 超声波工作距离兼容字段，单位 mm，编码为 `uint16` |
| `ultrasonic_adjust_distance_mm` | `100` | 超声波调节距离兼容字段，单位 mm，编码为 `uint16` |

这些整型参数在读取时会被夹紧到 `0..65535`。

## 编译

在 ROS 2 工作空间根目录执行：

```bash
colcon build --packages-select pusher_nav_bridge
source install/setup.bash
```

如果当前工作空间内还需要同时调试 LiDAR 可视化或串口硬件，可以一起编译相关包：

```bash
colcon build --packages-select lio_scan_monitor rs485_tester pusher_nav_bridge
source install/setup.bash
```

## 运行

### 使用 launch

```bash
source install/setup.bash
ros2 launch pusher_nav_bridge pusher_nav_bridge.launch.py
```

launch 文件会加载安装目录中的默认参数文件：

```text
install/pusher_nav_bridge/share/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

如果修改了源码目录下的 `config/pusher_nav_bridge_params.yaml`，需要重新 `colcon build` 后 launch 才会使用安装后的新配置。

### 直接指定参数文件

开发调试时可以直接指定源码目录中的参数文件：

```bash
source install/setup.bash
ros2 run pusher_nav_bridge pusher_nav_bridge_node \
  --ros-args \
  --params-file src/LidarBridge/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml
```

也可以临时覆盖关键参数：

```bash
source install/setup.bash
ros2 run pusher_nav_bridge pusher_nav_bridge_node \
  --ros-args \
  --params-file src/LidarBridge/pusher_nav_bridge/config/pusher_nav_bridge_params.yaml \
  -p poses_file:=$(pwd)/maps/poses.txt \
  -p follow_side:=left \
  -p travel_direction:=forward \
  -p target_left_distance_m:=1.10 \
  -p target_right_distance_m:=1.10
```

如果 `poses_file` 使用相对路径，先确认当前目录下存在该文件：

```bash
ls maps/poses.txt
```

## 运行日志

节点启动成功后会打印：

```text
Loaded <route_points> route points and <segments> segments from <poses_file>; dry-run protocol is enabled
```

如果还没有收到 odom，会以 2 秒节流频率打印：

```text
waiting for odometry
```

收到 odom 和 scan 后，会以 500 ms 节流频率打印 dry-run 状态：

```text
segment=1 s=2.50 side_detected=true dist=1.300 offset=0.200 confidence=0.82 payload=0x02 0x01 ...
```

字段含义：

| 字段 | 含义 |
|------|------|
| `segment` | 当前路线段编号，从 `1` 开始 |
| `s` | 当前机器人投影到路线上的累计弧长，单位 m |
| `side_detected` | 是否成功拟合出当前跟随侧护栏线 |
| `dist` | 当前跟随侧实测护栏距离，单位 m |
| `offset` | 当前跟随侧横向偏移，单位 m |
| `confidence` | 侧线拟合置信度，范围约为 `0..1` |
| `payload` | 26 字节业务 payload 的 Hex 字符串 |

如果 `side_detected=false`，payload 中的横向偏移字段会置 `0`，`reserved` 字段会置 `1`，用于提示当前侧距未被有效检测。

## 26 字节 payload

`encode_payload()` 输出固定 26 字节数组。多字节字段使用大端序编码。

| 字节偏移 | 字段 | 类型 | 单位/含义 |
|----------|------|------|-----------|
| `0` | 路径条数 | `uint8` | 编译后的路线段总数，超过 `255` 会夹紧到 `255` |
| `1` | 路径编号 | `uint8` | 当前路线段编号，超过 `255` 会夹紧到 `255` |
| `2..5` | 左侧距离 | `uint32` | 目标左侧延边距离，单位 mm |
| `6..7` | 横向距离 | `int16` | 当前跟随侧横向偏移，单位 mm，侧距无效时为 `0` |
| `8..11` | 右侧距离 | `uint32` | 目标右侧延边距离，单位 mm |
| `12..13` | 行走速度 | `uint16` | 单位 mm/s |
| `14..15` | 超声波工作距离 | `uint16` | 兼容字段，单位 mm |
| `16..17` | 超声波调节距离 | `uint16` | 兼容字段，单位 mm |
| `18` | 横向拐弯角度 | `uint8` | 当前段末端转角绝对值，单位 degree，超过 `255` 会夹紧到 `255` |
| `19` | 旋转类型 | `uint8` | `0` 不转，`1` 左转，`2` 右转，`3` 掉头，`4` 原地转 |
| `20..21` | 旋转速度 | `uint16` | 来自 `rotation_speed` |
| `22..25` | 预留字段 | `uint32` | 当前侧距有效时为 `0`，侧距无效时为 `1` |

当前 payload 只是业务字段区，不是完整 RS485 帧。接真实下位机前必须确认或补齐：

- 帧头
- 命令字
- 长度字段
- CRC 或校验和
- ACK 和超时重发策略
- 是否要求 Modbus RTU 格式
- 多字节字段端序
- `int16` 横向距离的正负号和补码解释
- 侧距无效时下位机如何处理 `reserved=1`

## 运行前检查

确认路线文件：

```bash
ls maps/poses.txt
```

确认话题存在：

```bash
ros2 topic list | grep -E '/fastlio2/lio_odom|/scan'
```

确认话题有数据：

```bash
ros2 topic hz /fastlio2/lio_odom
ros2 topic hz /scan
```

确认单帧 odom：

```bash
ros2 topic echo /fastlio2/lio_odom --once
```

如果话题名不一致，修改参数文件中的 `odom_topic` 和 `scan_topic`，或在运行时用 `-p` 覆盖。

## 调参建议

### 路线不稳定或段数过多

优先调整：

- 增大 `rdp_tolerance_m`，减少由噪声造成的碎段。
- 增大 `min_segment_length_m`，过滤短段。
- 增大 `heading_window_m`，让航向更平滑。
- 检查 `travel_direction` 是否和实际行驶方向一致。

### `side_detected=false`

按下面顺序排查：

1. 确认 `/scan` 坐标轴是否满足 `x` 前、`y` 左。
2. 确认 `follow_side` 是否和现场护栏方向一致。
3. 放宽 `side_roi_x_min_m`、`side_roi_x_max_m`，确保 ROI 覆盖护栏。
4. 放宽 `side_roi_y_min_m`、`side_roi_y_max_m`，确保侧向距离范围覆盖护栏。
5. 检查目标延边距离是否接近现场实测距离，必要时增大 `side_distance_gate_m`。
6. 降低 `min_side_inliers` 或增大 `ransac_inlier_threshold_m`，确认是否是点数不足或线拟合过严。
7. 增大 `line_heading_gate_deg`，确认是否是护栏线方向筛选过严。

### `dist` 正常但 `offset` 方向不符合下位机预期

先不要改侧距估计算法。应先和下位机确认纠偏方向定义，再在协议适配层对横向距离取反。

### `fatal error: cannot open poses file`

原因通常是 `poses_file` 相对路径的基准目录不对。处理方式：

```bash
ros2 run pusher_nav_bridge pusher_nav_bridge_node \
  --ros-args \
  -p poses_file:=/absolute/path/to/maps/poses.txt
```

或切换到包含 `maps/poses.txt` 的工作目录后再启动。

### 一直 `waiting for odometry`

说明节点没有收到 `odom_topic`。检查：

```bash
ros2 topic list
ros2 topic echo /fastlio2/lio_odom --once
```

如果 FAST-LIO 发布的话题不是 `/fastlio2/lio_odom`，修改 `odom_topic`。

### payload 有输出但下位机不能直接用

这是当前版本的预期行为。节点只打印 26 字节业务 payload，不写串口，也不生成完整通信帧。接真实 RS485 之前，需要新增串口发送模块和完整协议封装。

## 当前限制

- 不依赖 Navigation2，也不订阅规划路径。
- 不发送真实 RS485。
- 不包含完整串口帧格式、CRC、ACK 或重发策略。
- 不做 TF 转换，默认 `/scan` 已在车体坐标系。
- `poses.txt` 只使用 `x/y`，不会使用 `z` 或 quaternion。
- 侧距估计只输出当前 `follow_side` 的横向偏移。
- 超声波字段当前是兼容固定值，不读取真实超声波传感器。
- 路线在节点启动时编译一次，运行过程中不会自动重新加载 `poses_file`。

## 建议现场验证顺序

1. 确认 `maps/poses.txt` 的参考类型，配置 `poses_reference_type`。
2. 确认实际行驶方向，配置 `travel_direction`。
3. 确认跟随左侧还是右侧，配置 `follow_side`。
4. 配置 `target_left_distance_m` 和 `target_right_distance_m`。
5. 编译并启动 dry-run 节点。
6. 检查启动日志中的路线点数和路线段数。
7. 推动车辆或回放 bag，确认 `segment` 和 `s` 随路线前进变化。
8. 对照现场实际护栏距离，检查 `side_detected`、`dist`、`offset` 和 `confidence`。
9. 和下位机确认 26 字节 payload 字段。
10. 完成真实 RS485 帧格式后，再把 dry-run payload 接入串口发送。
