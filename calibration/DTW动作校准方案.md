# DTW 动作校准方案

本文说明当前预实验方案如何实现“动作 trial + DTW 模板匹配”。
## 1. 预实验目标

每位参与者在正式实验前完成个体化预实验。预实验收集该参与者的正确动作 trial，用于生成个人动作模板和 DTW 阈值。

当前采集两个动作：

| 动作 | 含义 |
|---|---|
| `HALF_RAISED` | 从起始位置到 Half-raised |
| `PICKING` | 从 Half-raised 到 Picking |

`NATURAL_DOWN` 不再作为单独动作 trial，而是作为 neutral pose / 起始基准的一部分。

## 2. 为什么用 DTW

DTW = Dynamic Time Warping，中文常译为“动态时间规整”。

它适合比较两段动作轨迹是否相似，即使两段动作速度不同。例如同样从 Half-raised 到 Picking，有人 1 秒完成，有人 2 秒完成。逐帧比较会受速度影响，DTW 会允许时间轴拉伸或压缩，更关注动作过程形状是否相似。

这符合 Word 文档中“3 到 6 次合格参考试验 + 个人动作模板 + 后续动作与个人模板比较”的方法。

## 3. Raw Data 组织

每位参与者一个文件夹：

```text
calibration/raw/participant_001/session.txt
```

`session.txt` 中保存完整串口输出即可，不需要每次 trial 单独复制。脚本会根据 marker 自动筛选。

必须包含的 marker：

```text
NEUTRAL_START
NEUTRAL_END
FLEX_STRAIGHT_START
FLEX_STRAIGHT_END
FLEX_BENT_START
FLEX_BENT_END
ACTION_SET,HALF_RAISED
TRIAL_START,HALF_RAISED,1
TRIAL_END,HALF_RAISED,1
TRIAL_ACCEPT,HALF_RAISED,1
TRIAL_REJECT,HALF_RAISED,2
```

只有 `TRIAL_ACCEPT` 的 trial 会进入计算。

## 4. DATA 格式

当前预实验 `.ino` 输出：

```text
DATA,timeMs,phase,trialId,action,
imu1Ax,imu1Ay,imu1Az,imu1Gx,imu1Gy,imu1Gz,
imu2Ax,imu2Ay,imu2Az,imu2Gx,imu2Gy,imu2Gz,
upperAngle,forearmAngle,
flexRaw,flexAngle,
pressureRaw,pressurePressed
```

说明：

- `imu1` = 上臂 IMU
- `imu2` = 前臂 IMU
- `upperAngle` = 上臂相对 neutral pose 的节段相对姿态角
- `forearmAngle` = 前臂相对 neutral pose 的节段相对姿态角
- `flexAngle` = Flex 基于伸直/弯曲基线映射后的角度
- `pressureRaw` 当前记录但不进入 DTW 主距离

## 5. DTW 特征

当前 DTW 使用 9 维特征：

```text
[
  imu1Ax, imu1Ay, imu1Az,
  imu2Ax, imu2Ay, imu2Az,
  upperAngle / 90,
  forearmAngle / 90,
  flexAngle / 90
]
```

陀螺仪数据用于 Madgwick filter 计算角度；当前 DTW 距离不直接使用 gyro。

Pressure 会保留在 raw data 里，后续可用于阶段规则，例如是否发生按压/抓取事件。

## 6. Madgwick 相对角度

当前 MPU6050 没有磁力计，因此不依赖 yaw。角度只用于 roll / pitch 相关的节段相对姿态变化。

计算流程：

1. 预实验开始时采集 neutral pose。
2. 记录上臂和前臂 IMU 的 neutral 四元数：

```text
qUpperNeutral
qForearmNeutral
```

3. 每帧使用 Madgwick filter 融合加速度和陀螺仪，得到：

```text
qUpperCurrent
qForearmCurrent
```

4. 计算相对姿态：

```text
qUpperRel   = inverse(qUpperNeutral)   * qUpperCurrent
qForearmRel = inverse(qForearmNeutral) * qForearmCurrent
```

5. 转成相对角度：

```text
relativeAngle = 2 * acos(clamp(qRel.w, -1, 1))
relativeAngleDeg = relativeAngle * 180 / PI
```

论文表述建议使用：

```text
节段相对姿态角 relative segment orientation angle
```

不要表述为精确的肩关节角或肘关节角。

## 7. DTW 计算

每个 accepted trial 会形成一个序列：

```text
T = [frame1, frame2, ..., frameN]
```

每个 frame 是 9 维特征。

两个 frame 的局部距离：

```text
d(a, b) = sum((a[j] - b[j])^2)
```

两段 trial 的 DTW：

```text
DTW[i][j] = d(ai, bj) + min(
  DTW[i-1][j],
  DTW[i][j-1],
  DTW[i-1][j-1]
)
```

最终距离：

```text
dtwDistance = DTW[n][m] / pathLength
```

除以 `pathLength` 是为了减少长动作天然距离更大的影响。

## 8. 阈值生成

每个动作建议收集 3 到 6 个 accepted trial。

对某个动作，脚本用留一法估计内部最近邻距离：

```text
nearestDistance(trial_i) = min(DTW(trial_i, other accepted trials))
```

然后：

```text
dtwThreshold = max(nearestDistance) * multiplier
```

当前默认：

```text
accepted trial 数量 <= 3: multiplier = 2.0
accepted trial 数量 > 3: multiplier = 1.5
```

## 9. Word 文档公式落地

Word 文档中需要保留的工程规则包括：

### 均值和标准差

```text
μx = sum(xi) / N
σx = sqrt(sum((xi - μx)^2) / (N - 1))
xlow = μx - kσx
xhigh = μx + kσx
k = 2
```

### 最小值、最大值及余量

```text
xlow_acc = min(xcorrect) - m
xhigh_acc = max(xcorrect) + m
```

第一版建议：

```text
角度 margin: 5 到 10 度
Flex angle margin: 5 到 10 度
Flex raw margin: 传感器量程 5% 到 10%
DTW margin: 内部最近邻最大距离乘以 1.5 或 2.0
```

这些是工程化容差，不应写成临床阈值。

### 百分位数范围

```text
xlow_acc = P5(xcorrect)
xhigh_acc = P95(xcorrect)
```

由于每个动作通常只有 3 到 6 个 accepted trial，百分位数第一版只作为可选分析指标，不作为主判定。

## 10. 正式实验关系

预实验完成后，电脑端脚本生成：

```text
calibration/dtw_threshold_report.csv
calibration/dtw_templates.json
main/DTWCalibrationData.h
```

正式实验阶段不重新计算阈值。ESP32 正式程序通过：

```cpp
#include "DTWMotionClassifier.h"
```

间接读取 `main/DTWCalibrationData.h` 中的个人模板和阈值。

为控制 ESP32 内存和计算量，脚本会把每个模板降采样到固定长度，默认最多 25 帧；正式程序也会把实时采样窗口降采样后再计算 DTW。DTW 不在每一帧完整重算，而是在动作采样窗口达到最小长度后间隔复核，并在动作片段切换/结束时再计算一次。
