# LidarBridge 流程与算法优化

本文档把推料机延边导航拆成可实现的离线流程和在线流程。主目标是：不依赖真实超声波，使用固定左/右目标延边距离、`poses.txt` 轨迹、FAST-LIO 定位和 LiDAR 两侧测距，生成稳定的路径段、转角、横向偏移和 RS485 下发数据。

## 总体原则

1. `左侧距离` 和 `右侧距离` 是目标延边距离，只来自任务配置。
2. LiDAR 提供左右实测距离和偏移，不改写目标距离。
3. `poses.txt` 负责路线几何：前进方向、左右扩展线、段里程、转角方向。
4. FAST-LIO 位姿负责在线定位到路线位置和段切换，不直接当侧距使用。
5. 超声波字段只作为协议兼容固定值；真实超声波可以不接。

## 推荐模块拆分

```text
RouteCompiler
  输入: poses.txt + 任务配置
  输出: 中心线、左右虚拟护栏线、路径段、转角、静态协议字段

RouteLocalizer
  输入: FAST-LIO x/y/yaw + RouteCompiler 输出
  输出: 当前段 id、路线弧长 s、路线横向误差、是否接近转弯点

SideDistanceEstimator
  输入: /scan + TF + 当前路线方向 + 目标侧距
  输出: 左/右实测距离、偏移、护栏姿态角、置信度

PusherProtocolBridge
  输入: 当前段配置 + SideDistanceEstimator 输出
  输出: RS485 payload / dry-run Hex / 发送状态
```

## 离线流程：轨迹编译

离线流程在启动任务或切换地图时执行一次。

```text
LOAD_POSES
  -> CLEAN_POINTS
  -> APPLY_TRAVEL_DIRECTION
  -> RESAMPLE_BY_ARCLENGTH
  -> SMOOTH_HEADING
  -> BUILD_NORMALS
  -> BUILD_LEFT_RIGHT_EDGE_LINES
  -> EXTRACT_SEGMENTS_AND_TURNS
  -> BUILD_STATIC_PROTOCOL_FIELDS
```

### 1. 读取和清洗 `poses.txt`

支持格式：

```text
frame.pcd x y z qw qx qy qz
```

清洗规则：

- 跳过空行、注释行、无法解析行。
- 只使用有限的 `x/y`。
- 距离小于 `min_point_gap_m` 的点合并或删除，建议 `0.05~0.15 m`。
- 如果 `travel_direction=reverse`，先反转点序，再做后续计算。

伪代码：

```text
points = parse_poses_xy(file)
points = drop_invalid(points)
points = merge_close_points(points, min_point_gap_m)
if travel_direction == reverse:
  points = reverse(points)
```

### 2. 按弧长重采样

原始关键帧间距不稳定，直接用相邻点算方向会抖。建议按弧长重采样：

```text
resample_step_m = 0.20 ~ 0.50
```

输出每个点的累计弧长：

```text
P_i = (x_i, y_i)
s_i = sum(distance(P_k, P_{k-1}))
```

优化点：后续所有段切换都使用 `s_i`，不要用点编号。

### 3. 平滑前进方向

推荐用窗口中心差分，而不是单个相邻点：

```text
heading_window_m = 0.5 ~ 1.0
P_prev = route point around s_i - heading_window_m
P_next = route point around s_i + heading_window_m
t_i = normalize(P_next - P_prev)
psi_i = atan2(t_i.y, t_i.x)
```

端点使用单侧差分。计算完所有 `psi_i` 后做 angle unwrap，避免 `+pi` 和 `-pi` 跳变。

### 4. 左右法线与路线扩展

按实际前进方向计算左右法线：

```text
n_left_i  = (-sin(psi_i),  cos(psi_i))
n_right_i = ( sin(psi_i), -cos(psi_i))
```

默认 `poses_reference_type=vehicle_center`，即 `poses.txt` 是车体中心线：

```text
left_edge_i  = P_i + target_left_distance_m  * n_left_i
right_edge_i = P_i + target_right_distance_m * n_right_i
```

如果 `poses_reference_type=left_guardrail`：

```text
center_i = P_i - target_left_distance_m * n_left_i
```

如果 `poses_reference_type=right_guardrail`：

```text
center_i = P_i - target_right_distance_m * n_right_i
```

优化点：所有左右扩展都必须在 `travel_direction` 已经应用之后计算，否则反向行驶时左右会反。

## 路径分段优化

推荐使用“RDP 折线简化 + 方向窗口验证”，比单纯按相邻点曲率更稳。

### 1. RDP 折线简化

对重采样中心线运行 Ramer-Douglas-Peucker 简化：

```text
rdp_tolerance_m = 0.10 ~ 0.30
corners = rdp(centerline, rdp_tolerance_m)
```

RDP 输出的折线顶点可以作为候选转弯点。这样能避免局部噪声把直线切碎。

### 2. 建立直行段

对相邻 RDP 顶点形成直行段：

```text
segment.start_s
segment.end_s
segment.length_m = end_s - start_s
segment.heading_rad = atan2(B.y - A.y, B.x - A.x)
```

过滤过短段：

```text
min_segment_length_m = 0.5 ~ 1.0
```

过短段可以合并到前后段，防止下位机频繁切换路径编号。

### 3. 计算转角和旋转类型

相邻两段：

```text
turn_angle_rad = normalize_angle(next.heading_rad - current.heading_rad)
turn_angle_deg = turn_angle_rad * 180 / pi
```

分类：

```text
abs(turn_angle_deg) < 5       -> NONE
turn_angle_deg > 0            -> LEFT
turn_angle_deg < 0            -> RIGHT
abs(turn_angle_deg) > 150     -> U_TURN 或 IN_PLACE
```

协议字段：

```text
横向拐弯角度 = clamp(round(abs(turn_angle_deg)), 0, 255)
旋转类型 = NONE / LEFT / RIGHT / U_TURN / IN_PLACE
```

优化点：转角方向来自反转后的路线段方向，不来自原始 quaternion。

### 4. 段切换条件

在线运行时不要只用“距离某个点很近”切段。推荐投影到当前段得到路线弧长：

```text
projection = project(robot_xy, current_segment)
s_robot = segment.start_s + projection.along_segment
e_route = projection.cross_track_error
```

切段条件：

```text
s_robot >= segment.end_s - switch_margin_m
并且 heading_error_abs < switch_heading_gate_deg
并且 next segment 已经稳定可用
```

推荐参数：

```text
switch_margin_m = 0.2 ~ 0.5
switch_heading_gate_deg = 45
```

加滞回保护：

- 一旦切到下一段，不允许立即切回上一段。
- 只有当 `s_robot` 连续多帧超过阈值才切段。
- 定位跳变时暂停切段，只保持当前段或降速。

## 在线流程：运行闭环

在线流程按 10~30 Hz 跑，发送频率按下位机能力设置。

```text
WAIT_LOCALIZATION
  -> LOCATE_ON_ROUTE
  -> ESTIMATE_SIDE_DISTANCE
  -> FILTER_AND_VALIDATE_OFFSET
  -> BUILD_RUNTIME_FRAME
  -> SEND_OR_DRY_RUN
  -> CHECK_SEGMENT_SWITCH
```

### 1. 路线定位

用 FAST-LIO 当前 `x/y/yaw` 找最近段：

```text
candidate_segments = current segment + neighbor segments
best_projection = min distance projection
```

输出：

```text
current_segment_id
s_robot
route_cross_track_error_m
route_heading_error_rad
```

`route_cross_track_error_m` 可用于诊断和安全保护，但不要直接替代 LiDAR 侧距。

### 2. LiDAR 侧距估计

推荐使用“路线方向约束 + 侧向 ROI + 期望距离门控 + RANSAC 线拟合”。

#### ROI 选择

先把 `/scan` 点转换到 `base_link`。按跟随侧选 ROI：

```text
x_min <= x <= x_max
y_min <= abs(y) <= y_max
side_sign * y > 0
```

推荐初值：

```text
x_min = -2.0
x_max =  3.0
y_min =  0.2
y_max =  5.0
```

#### 期望距离门控

因为目标距离已知，可以优先保留接近目标距离的点：

```text
expected = target_distance_m
gate = side_distance_gate_m
keep if abs(abs(y) - expected) <= gate
```

推荐：

```text
side_distance_gate_m = 0.5 ~ 1.0
```

如果门控后点太少，再退回普通 ROI。这样可以减少远处墙体或临时障碍误入护栏线。

#### RANSAC 拟合

输入 ROI 点，RANSAC 拟合直线：

```text
ransac_iterations = 50 ~ 100
ransac_inlier_threshold_m = 0.05 ~ 0.10
min_inliers = 8 ~ 20
```

拟合后校验：

- 内点数量足够。
- 内点比例足够。
- 平均残差低。
- 内点平均 `y` 在正确侧。
- 线方向与车体前进方向或路线方向夹角不超过阈值。

推荐方向阈值：

```text
line_heading_gate_deg = 45
```

#### 距离和姿态误差

线模型：

```text
line: p = p0 + k * u
u = unit direction
distance_m = abs(cross(u, p0))
line_heading_rad = atan2(u.y, u.x)
guardrail_heading_error_rad = normalize_to_half_pi(line_heading_rad)
```

如果使用路线方向约束，也可以计算：

```text
guardrail_heading_error_rad = normalize_to_half_pi(line_heading_rad - route_heading_in_base)
```

### 3. 偏移计算

左侧：

```text
left_offset_m = left_distance_m - target_left_distance_m
```

右侧：

```text
right_offset_m = -(right_distance_m - target_right_distance_m)
```

当前跟随侧：

```text
active_offset_m = follow_side == left ? left_offset_m : right_offset_m
```

发送到协议前量化：

```text
offset_mm = clamp(round(active_offset_m * 1000), -32768, 32767)
```

如果下位机不支持 int16 补码，就需要把方向放到额外标志位，或约定 `横向距离` 永远为绝对值、由 `旋转类型/纠偏方向` 表达符号。

### 4. 滤波优化

推荐顺序：

```text
raw offset
  -> 跳变门控
  -> 3~5 帧中值滤波
  -> IIR 低通
  -> 发送限幅
```

IIR：

```text
alpha = dt / (tau + dt)
filtered = filtered_prev + alpha * (median - filtered_prev)
```

推荐：

```text
tau = 0.15 ~ 0.30 s
max_offset_jump_m = 0.30 ~ 0.50
max_send_offset_m = 1.00
```

### 5. 置信度和降级

置信度建议：

```text
confidence =
  w1 * inlier_count_score +
  w2 * inlier_ratio_score +
  w3 * residual_score +
  w4 * side_consistency_score +
  w5 * time_freshness_score
```

简单实现可分三级：

| 等级 | 条件 | 行为 |
|------|------|------|
| GOOD | 内点多、残差低、时间新鲜 | 正常发送偏移 |
| WEAK | 点少或残差偏高 | 限幅发送，降低速度或保持上一帧 |
| LOST | 超时或拟合失败 | 偏移置 0/保持上一帧/请求停车 |

推荐超时：

```text
lidar_stale_timeout_s = 0.3
localization_stale_timeout_s = 0.3
```

## RS485 下发流程优化

建议区分静态段配置和动态纠偏。

### 1. 段配置帧

在进入新路径段时发送：

- 路径条数
- 路径编号
- 固定目标左侧距离
- 固定目标右侧距离
- 行走速度
- 超声波兼容固定值
- 横向拐弯角度
- 旋转类型
- 旋转速度
- 预留字段

### 2. 动态纠偏帧

如果下位机允许周期更新同一 payload，则按 `10~20 Hz` 发送当前 `横向距离`。如果下位机只接受段配置，则进入段时先发 0，后续需要新增实时状态帧。

推荐发送策略：

```text
send_period_ms = 50 ~ 100
only_send_if_changed = true
min_offset_delta_mm = 5 ~ 10
watchdog_timeout_ms = 200 ~ 300
```

### 3. payload 填充建议

| 字段 | 数据来源 |
|------|----------|
| 路径条数 | RouteCompiler 段数量 |
| 路径编号 | RouteLocalizer 当前段 |
| 左侧距离 | `target_left_distance_m` 量化 |
| 横向距离 | `active_offset_m` 量化 |
| 右侧距离 | `target_right_distance_m` 量化 |
| 行走速度 | 当前段配置 |
| 超声波工作距离 | 固定兼容值 |
| 超声波调节距离 | 固定兼容值 |
| 横向拐弯角度 | 当前段末端转角绝对值 |
| 旋转类型 | 当前段末端转向枚举 |
| 旋转速度 | 当前段配置 |
| 预留字段 | 0 或版本/状态位 |

## 推荐状态机

```text
INIT
  读取配置，打开串口，等待 TF/odom/scan

COMPILE_ROUTE
  解析 poses.txt，重采样，分段，计算左右扩展线和转角

SEND_SEGMENT
  下发当前段静态配置

FOLLOW
  定位到路线 s，估计 LiDAR 侧距，滤波偏移，周期发送横向距离

APPROACH_TURN
  接近段尾，提前降速或准备旋转参数

TURNING
  发送旋转类型/角度/速度，等待下位机完成或按定位/里程门限判断完成

REACQUIRE_SIDE
  转弯后重新捕获左/右护栏线，置信度恢复后进入下一段 FOLLOW

SAFE_HOLD
  定位丢失、LiDAR 丢失、串口失败、偏移异常时保持/停车/降速
```

状态转换关键条件：

- `COMPILE_ROUTE -> SEND_SEGMENT`：路径段数大于 0。
- `SEND_SEGMENT -> FOLLOW`：发送成功或 dry-run 验证通过。
- `FOLLOW -> APPROACH_TURN`：`s_robot >= segment.end_s - turn_prepare_distance_m`。
- `APPROACH_TURN -> TURNING`：达到段尾或下位机请求转向。
- `TURNING -> REACQUIRE_SIDE`：旋转角达到、定位方向匹配或下位机 ACK。
- `REACQUIRE_SIDE -> FOLLOW`：LiDAR 侧线置信度恢复。
- 任意状态 -> `SAFE_HOLD`：定位超时、LiDAR 超时、串口错误、偏移超限。

## 参数初值建议

| 参数 | 建议值 |
|------|--------|
| `min_point_gap_m` | `0.05 ~ 0.15` |
| `resample_step_m` | `0.20 ~ 0.50` |
| `heading_window_m` | `0.5 ~ 1.0` |
| `rdp_tolerance_m` | `0.10 ~ 0.30` |
| `min_segment_length_m` | `0.5 ~ 1.0` |
| `switch_margin_m` | `0.2 ~ 0.5` |
| `turn_prepare_distance_m` | `0.5 ~ 1.0` |
| `x_min/x_max` | `-2.0 / 3.0` |
| `y_min/y_max` | `0.2 / 5.0` |
| `side_distance_gate_m` | `0.5 ~ 1.0` |
| `ransac_inlier_threshold_m` | `0.05 ~ 0.10` |
| `line_heading_gate_deg` | `45` |
| `lidar_stale_timeout_s` | `0.3` |
| `tau` | `0.15 ~ 0.30` |
| `send_period_ms` | `50 ~ 100` |

## 最小可实现版本

当前 `pusher_nav_bridge` 已完成第一版最小闭环：

1. `poses.txt` 解析、反向、弧长重采样。
2. 用 RDP 生成路径段和转角。
3. 用侧向 ROI + 确定性 RANSAC 做侧线拟合。
4. 按当前跟随侧计算 `active_offset_m`。
5. dry-run 打印 payload，并通过单元测试确认字段量化正确。

下一版建议升级：

1. 增加时间连续滤波和失效保护状态机。
2. RViz 显示中心线、左右扩展线、拟合护栏线和偏移箭头。
3. 接入真实 RS485 和下位机 ACK。
4. 根据现场纠偏方向确认 `横向距离` 是否需要协议层取反。

## 验证用例

| 用例 | 预期 |
|------|------|
| 直线路径 | 只生成一个直行段，转角为 0 |
| 90 度左转 | `旋转类型=LEFT`，角度接近 90 |
| 90 度右转 | `旋转类型=RIGHT`，角度接近 90 |
| 反向行驶 | 左右法线和转角方向按实际前进方向重新计算 |
| 左侧目标 1.1m，实测 1.3m | `active_offset=+0.2m` |
| 右侧目标 1.1m，实测 1.3m | `active_offset=-0.2m` |
| LiDAR 丢失超过 0.3s | 进入 WEAK/LOST 或 SAFE_HOLD |
| 定位跳变 | 暂停切段，保持当前段或降速 |
| RS485 发送失败 | 进入 SAFE_HOLD 或重试 |
