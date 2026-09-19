# BLE Device: Yanteon-JDY16HOST0

记录日期：2026-07-08  
记录位置：`/home/tensorlab`

## 基本信息

| 项目 | 值 |
| --- | --- |
| 设备名称 | `Yanteon-JDY16HOST0` |
| 蓝牙地址 | `11:89:88:11:A1:0C` |
| 地址类型 | `public` |
| BlueZ 设备路径 | `/org/bluez/hci0/dev_11_89_88_11_A1_0C` |
| 本机蓝牙适配器 | `hci0` |
| 本机控制器地址 | `DC:4A:9E:E0:93:59` |
| 配对状态 | `Paired: no` |
| 信任状态 | `Trusted: yes` |
| 阻止状态 | `Blocked: no` |
| 传统配对 | `LegacyPairing: no` |

## 扫描结果

扫描时确认目标设备存在：

```text
Device 11:89:88:11:A1:0C
Name: Yanteon-JDY16HOST0
RSSI: -33
ManufacturerData Key: 0x801c
ManufacturerData Value: 88 a0 11 89 88 11 a1 0c
```

## 广播 / 设备 UUID

`bluetoothctl info 11:89:88:11:A1:0C` 显示的 UUID：

```text
00001800-0000-1000-8000-00805f9b34fb  Generic Access Profile
00001801-0000-1000-8000-00805f9b34fb  Generic Attribute Profile
0000ffe0-0000-1000-8000-00805f9b34fb  Unknown / vendor service
```

## GATT 服务和特征

成功连接并解析服务后枚举到：

```text
service: 00001801-0000-1000-8000-00805f9b34fb
  characteristic: 00002a05-0000-1000-8000-00805f9b34fb
  flags: indicate

service: 0000ffe0-0000-1000-8000-00805f9b34fb
  characteristic: 0000ffe1-0000-1000-8000-00805f9b34fb
  path: /org/bluez/hci0/dev_11_89_88_11_A1_0C/service000e/char000f
  flags: write-without-response, write, notify

  characteristic: 0000ffe2-0000-1000-8000-00805f9b34fb
  path: /org/bluez/hci0/dev_11_89_88_11_A1_0C/service000e/char0012
  flags: write-without-response, write, notify

  characteristic: 0000ffe3-0000-1000-8000-00805f9b34fb
  path: /org/bluez/hci0/dev_11_89_88_11_A1_0C/service000e/char0015
  flags: write-without-response, write, notify
```

## 发送信息时使用的地址

连接设备地址：

```text
11:89:88:11:A1:0C
```

优先尝试写入特征 UUID：

```text
0000ffe1-0000-1000-8000-00805f9b34fb
```

备选可写特征 UUID：

```text
0000ffe2-0000-1000-8000-00805f9b34fb
0000ffe3-0000-1000-8000-00805f9b34fb
```

## 发送示例

使用当前脚本发送十六进制数据：

```bash
python3 ble_inspect_jdy16.py \
  --address 11:89:88:11:A1:0C \
  --char-uuid 0000ffe1-0000-1000-8000-00805f9b34fb \
  --write-hex "01 02 03"
```

## 注意

脚本中的：

```python
REQUESTED_UUID = "0000000f-0000-1000-8000-00805f9b34fb"
```

不是该设备枚举出的实际 GATT UUID。`000f` 更像是 BlueZ 对象路径中的 `char000f` 编号；该路径对应的真实特征 UUID 是：

```text
0000ffe1-0000-1000-8000-00805f9b34fb
```
