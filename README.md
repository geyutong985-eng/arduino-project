# Arduino 智能硬件项目

智能硬件基础作业代码库

## 项目简介

这是一个 Arduino 智能硬件项目，用于组内协作开发。

## 协作要求

> ⚠️ 所有组员必须按以下流程操作

1. **创建自己的分支**：每人一个分支，用名字命名（如 `zhangsan`）
2. **在分支上开发**：所有代码改动都在自己分支上完成
3. **提交 PR**：完成后在 GitHub 上创建 Pull Request
4. **等待合并**：组长审核后合并到 main

详细操作步骤见 [GitHub 协作指南](./COLLABORATION_GUIDE.md)

## 快速开始

1. 安装 [Arduino IDE](https://www.arduino.cc/en/software) 或 [Arduino CLI](https://github.com/arduino/arduino-cli)
2. 用 Arduino IDE 打开 `.ino` 文件
3. 选择开发板型号，连接设备
4. 点击上传

## 目录结构

- `main.ino` - 主循环
- `SensorIMU.h/cpp` - IMU姿态传感器读取
- `SensorEMG.h/cpp` - EMG肌电传感器读取
- `SensorPPG.h/cpp` - PPG脉搏传感器读取
- `SensorFlex.h/cpp` - 弯曲传感器读取
- `SensorPressure.h/cpp` - 压力传感器读取
- `SensorVibrate.h/cpp` - 震动马达控制
- `UnityComm.h/cpp` - Unity通信

## 旧文件（待删除）

- `helloworld.ino` - Hello World 示例（可删除）
- `distance_led.ino` - 超声波测距示例（可删除）

## 许可证

待定