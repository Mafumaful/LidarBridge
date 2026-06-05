# rs485_tester

`rs485_tester` 是 RS485 串口通信测试包，用于验证串口设备、波特率、数据位、校验位、停止位、RS485 硬件模式和基础字节发送是否正常。

它不是最终的推料机路径业务协议。后续推料机导航桥接节点可以复用底层串口打开和写入逻辑，但协议打包应单独实现。

## 功能

- 从 YAML 配置读取串口参数。
- 使用 `termios` 打开串口设备。
- 可选通过 `TIOCSRS485` 打开 Linux RS485 硬件模式。
- 构造 Modbus RTU 测试请求并自动追加 CRC16。
- 支持 `read_holding_registers`、`write_single_register`、`write_multiple_registers` 三种测试请求。

## Modbus 功能码

`rs485_tester` 当前构造的是 Modbus RTU 主机请求。默认配置：

```yaml
request_type: "read_holding_registers"
```

这个请求对应 **03 功能码**，作用是读取从机保持寄存器的值，不是写寄存器。

三种请求类型和功能码的对应关系如下：

| `request_type` | 功能码 | Modbus 含义 | 主要参数 |
|----------------|--------|-------------|----------|
| `read_holding_registers` | `0x03` | 读取从机保持寄存器 | `slave_id`、`start_address`、`register_count` |
| `write_single_register` | `0x06` | 写单个保持寄存器 | `slave_id`、`start_address`、`write_value` |
| `write_multiple_registers` | `0x10` | 写多个保持寄存器 | `slave_id`、`start_address`、`write_values` |

因此，如果只是验证从机某段寄存器是否可读，使用 `0x03`。如果要测试写入，单个寄存器使用 `0x06`，多个连续寄存器使用 `0x10`。

## 和推料机协议的关系

推料机导航协议应按下位机字段表打包，例如：

- 路径条数
- 路径编号
- 固定目标左侧延边距离
- LiDAR 横向偏移
- 固定目标右侧延边距离
- 行走速度
- 超声波兼容固定值
- 横向拐弯角度
- 旋转类型
- 旋转速度
- 预留字段

这些字段不应默认当成通用 Modbus 寄存器发送，除非下位机明确要求使用 Modbus RTU。建议新增 `PusherPathProtocol` 或 `pusher_nav_bridge` 节点专门完成业务帧序列化和 dry-run 反向解析。

## 编译

在 ROS 2 工作空间根目录执行：

```bash
colcon build --packages-select rs485_tester
source install/setup.bash
```

## 运行

```bash
ros2 launch rs485_tester rs485_test.launch.py
```

修改 `config/rs485_params.yaml` 可调整串口和测试请求。

## 配置

常用串口参数：

- `device`
- `baud_rate`
- `data_bits`
- `parity`
- `stop_bits`
- `flow_control`
- `enable_rs485_mode`
- `rs485_delay_before_send_us`
- `rs485_delay_after_send_us`
- `rs485_rx_during_tx`

Modbus RTU 测试参数：

- `slave_id`：目标站地址
- `request_type`：测试请求类型。`read_holding_registers` 对应 `0x03` 读保持寄存器，`write_single_register` 对应 `0x06` 写单个寄存器，`write_multiple_registers` 对应 `0x10` 写多个寄存器
- `start_address`：寄存器起始地址
- `register_count`：读保持寄存器时使用
- `write_value`：写单个寄存器时使用
- `write_values`：写多个寄存器时使用，逗号分隔 16 位值
