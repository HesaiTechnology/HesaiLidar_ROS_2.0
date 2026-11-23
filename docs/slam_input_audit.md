# Add offline cloud timing and ring-field auditing for SLAM inputs

SLAM integration needs a compact report that separates point-field compatibility, cloud timing and nearby IMU timing. This would make XT16 integration and recorded-data checks easier to reproduce.

## Run
```bash
python3 tools/slam_input_audit.py examples/slam_audit/clouds.jsonl --imu examples/slam_audit/imu.jsonl
```

## Behavior
- Validate exported ring:uint16 and timestamp:float64 declarations and configured ring bounds.
- Check finite point timestamps, nonincreasing frame stamps, frame spans and header/point time differences.
- Compare the minimum and maximum point timestamps with their nearest IMU samples.
- Stream cloud JSONL records and output counts, worst observed intervals and per-frame rejection reasons.

## Input and result contract
The first version consumes exported JSONL records. The exporter supplies field types and a common time domain; endpoint checks measure temporal proximity, while IMU continuity and extrinsic calibration remain separate checks. It uses synthetic fixtures.
Cloud JSONL records contain stamp (seconds), fields {ring:"uint16",timestamp:"float64"}, and points as [ring,timestamp] pairs. IMU JSONL records contain stamp. IMU stamps must be strictly increasing. Defaults: 16 rings, maximum frame span/header discrepancy 0.2 s, maximum nearest-IMU gap 0.02 s. All thresholds are configurable. Point-array time ordering is unrestricted. This checks exported schema declarations; a ROS/rosbag exporter is a separate adapter. Cloud frames are streamed; IMU stamps and problem records are held in memory. Exit 0=pass, 1=findings, 2=input error.

## Tests
12 Python unit tests. See the tests directory.

---

# 新增面向 SLAM 输入的点云时间与线号离线诊断

SLAM 集成需要把点字段兼容性、点云时间和邻近 IMU 时间分开检查，形成便于复核的报告，方便 XT16 接入和录制数据检查。

## 运行
```bash
python3 tools/slam_input_audit.py examples/slam_audit/clouds.jsonl --imu examples/slam_audit/imu.jsonl
```

## 功能
- 核对导出的 ring:uint16、timestamp:float64 字段声明和配置的线号范围。
- 检查逐点时间有限性、帧时间倒退／重复、帧时间跨度和帧头／点时间差。
- 将每帧最早、最晚点时间与最近 IMU 样本比较。
- 逐帧读取点云 JSONL，报告帧数、点数、最大时间差和逐帧问题。

## 输入与结果
首版读取导出的 JSONL 记录。导出端提供字段类型和统一时间域；本工具检查帧两端与 IMU 的时间接近程度，IMU 连续性及外参标定由对应检查处理。样例采用合成数据。
点云 JSONL 每条包含 stamp（秒）、fields {ring:"uint16",timestamp:"float64"}、points（[ring,timestamp] 数组）。IMU JSONL 每条包含 stamp，须严格递增。默认 16 线、帧跨度／帧头时间差上限 0.2 秒、最近 IMU 时间差上限 0.02 秒，均可配置；不要求点数组按时间排序。检查使用导出的字段声明，ROS／rosbag 导出端作为独立适配器接入。点云逐帧读取，IMU 时间列表及问题记录保留在内存。退出码：0 通过，1 有问题，2 输入错误。

## 测试
12 项 Python 单元测试，详见 tests 目录。
