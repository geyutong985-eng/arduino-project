# 预实验校准说明

本文件夹用于正式实验前的个体化预实验校准。当前方案已经从“3 个静态姿态阈值”改为“2 个动作 trial + DTW 模板 + Flex + 相对角度”。

## 当前流程

1. 使用独立预实验程序采集数据：

```text
pretest_calibration/pretest_calibration.ino
```

2. 每位参与者一个 raw data 文件夹：

```text
calibration/raw/participant_001/session.txt
calibration/raw/participant_002/session.txt
```

3. 每个 `session.txt` 里粘贴该参与者完整串口输出，包括：

- neutral pose
- Flex 伸直基线
- Flex 弯曲基线
- `HALF_RAISED` 动作 trial
- `PICKING` 动作 trial
- `TRIAL_ACCEPT` / `TRIAL_REJECT` 标记

4. 运行 DTW 计算脚本：

```powershell
.\tools\compute_dtw_thresholds.ps1 -SessionFile .\calibration\raw\participant_001\session.txt
```

5. 脚本会自动只使用合格 trial，输出：

```text
calibration/dtw_threshold_report.csv
calibration/dtw_templates.json
main/DTWCalibrationData.h
```

`main/DTWCalibrationData.h` 会被正式实验程序 `main/main.ino` 直接 include。跑完脚本后不需要手动改这个头文件，直接重新上传正式实验程序即可。

如果 PowerShell 拦截脚本执行，使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\compute_dtw_thresholds.ps1 -SessionFile .\calibration\raw\participant_001\session.txt
```

`tools/compute_imu_thresholds.ps1` 是旧的静态姿态阈值脚本，用来从 `natural_down.txt`、`half_raised.txt`、`picking.txt` 生成 `main/IMUCalibrationData.h`。当前 DTW 预实验主流程使用 `compute_dtw_thresholds.ps1`。

## 当前采集动作

| 编号 | 动作 | 含义 |
|---|---|---|
| `1` | `HALF_RAISED` | 从起始位置到 Half-raised |
| `2` | `PICKING` | 从 Half-raised 到 Picking |

`NATURAL_DOWN` 不再作为一个单独动作 trial，而是作为 neutral pose / 起始基准的一部分。

## 文档入口

- [[完整实验流程教程]]：从预实验采集、PowerShell 运行脚本，到上传正式程序的完整教程
- [[预实验操作说明]]：实验员怎么操作串口命令、怎么保存数据
- [[DTW动作校准方案]]：DTW、Madgwick 相对角度、min/max + margin 等方法说明
- [[计算方法说明]]：当前计算脚本具体怎么算

## 旧文件说明

`calibration/raw/` 里可能还保留早期静态姿态方案的兼容文件。当前正式预实验不要使用这些文件，请只使用每位参与者自己的：

```text
calibration/raw/participant_xxx/session.txt
```
