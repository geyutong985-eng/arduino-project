// ===== IMU 姿态传感器模块（Madgwick滤波）=====
// 协议：帧头 0x7E 0x23，小端存储
// 软串口：D2(TX) D3(RX)
// 手势识别：自然下垂→半抬→摘果子

#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <MadgwickAHRS.h>

// ===== 协议帧头和功能码 =====
#define FRAME_HEAD1 0x7E
#define FRAME_HEAD2 0x23
#define IMU_FUNC_RAW_ACCEL 0x04  // 原始数据
#define IMU_FUNC_QUAT      0x16  // 四元数
#define IMU_FUNC_EULER     0x26  // 欧拉角

// ===== 解析状态 =====
enum {
    RX_STATE_EXPECT_HEAD1 = 0,
    RX_STATE_EXPECT_HEAD2,
    RX_STATE_EXPECT_LENGTH,
    RX_STATE_EXPECT_FUNCTION,
    RX_STATE_COLLECT_DATA
};

// ===== 手势识别状态 =====
enum ArmState {
    ARM_STATE_UNKNOWN = 0,
    ARM_STATE_NATURAL_DOWN,  // 自然下垂
    ARM_STATE_HALF_RAISED,   // 半抬
    ARM_STATE_PICKING        // 摘果子
};

class SensorIMU {
private:
    SoftwareSerial imuSerial;   // RX=D3, TX=D2

    // 解析状态机
    uint8_t rxState;
    uint8_t frameLength;
    uint8_t frameFunction;
    uint8_t frameBuffer[64];
    uint16_t frameIndex;
    uint16_t payloadLength;

    // 解析后的数据
    float ax, ay, az;   // 加速度(g)
    float gx, gy, gz;   // 角速度(rad/s)
    float roll, pitch, yaw;  // 姿态角(度)

    // Madgwick 滤波器
    Madgwick filter;

    // 手势识别状态
    ArmState currentArmState;
    ArmState lastArmState;
    bool passedNatural;
    bool passedHalf;
    bool pickTriggered;
    unsigned long stateHoldStart;
    unsigned long pickSignalStart;

public:
    SensorIMU() : imuSerial(3, 2) {
        roll = pitch = yaw = 0.0f;
        ax = ay = az = 0.0f;
        gx = gy = gz = 0.0f;

        rxState = RX_STATE_EXPECT_HEAD1;
        frameLength = 0;
        frameFunction = 0;
        frameIndex = 0;
        payloadLength = 0;

        filter.begin(50);  // 50Hz

        currentArmState = ARM_STATE_UNKNOWN;
        lastArmState = ARM_STATE_UNKNOWN;
        passedNatural = false;
        passedHalf = false;
        pickTriggered = false;
        stateHoldStart = 0;
        pickSignalStart = 0;
    }

    // 初始化
    void init() {
        reset();
        imuSerial.begin(115200);
        Serial.println("[IMU] serial begin: 115200");
    }

    // 请求输出数据（初始化后调用）
    void requestEuler() {
        // 重置用户数据
        uint8_t cmdReset[] = {0x7E, 0x23, 0x07, 0xA0, 0x01, 0x5F, 0xE8};
        imuSerial.write(cmdReset, 7);
        delay(500);

        // 设置输出频率 50Hz
        uint8_t cmdFreq[] = {0x7E, 0x23, 0x07, 0x60, 0x32, 0x5F, 0xD9};
        imuSerial.write(cmdFreq, 7);
        delay(100);
        Serial.println("[IMU] commands sent");
    }

    // 重置解析状态
    void reset() {
        rxState = RX_STATE_EXPECT_HEAD1;
        frameLength = 0;
        frameFunction = 0;
        frameIndex = 0;
        payloadLength = 0;
    }

    // 更新（主循环调用）
    void update() {
        while (imuSerial.available()) {
            receiveByte(imuSerial.read());
        }
    }

    // ===== 手势识别 =====
    ArmState classifyArmState(float roll, float pitch, float yaw,
                               float gx, float gy, float gz) {
        // 稳定性判断
        bool stable = (abs(gx) < 0.50f && abs(gy) < 0.50f && abs(gz) < 0.50f);
        if (!stable) return ARM_STATE_UNKNOWN;

        // 自然下垂: roll>50, pitch在-35~5之间
        if (roll > 50.0f && pitch > -35.0f && pitch < 5.0f) {
            return ARM_STATE_NATURAL_DOWN;
        }

        // 半抬: roll在-40~20之间, pitch<-40
        if (roll > -40.0f && roll < 20.0f && pitch < -40.0f) {
            return ARM_STATE_HALF_RAISED;
        }

        // 摘果子: roll<-50, pitch在-30~10之间
        if (roll < -50.0f && pitch > -30.0f && pitch < 10.0f) {
            return ARM_STATE_PICKING;
        }

        return ARM_STATE_UNKNOWN;
    }

    // 检测摘果子动作（需在loop中调用）
    void detectPickAction() {
        unsigned long now = millis();

        ArmState detected = classifyArmState(roll, pitch, yaw, gx, gy, gz);
        currentArmState = detected;

        if (currentArmState != lastArmState) {
            stateHoldStart = now;
            lastArmState = currentArmState;
        }

        // 第一步：自然下垂
        if (currentArmState == ARM_STATE_NATURAL_DOWN) {
            passedNatural = true;
            passedHalf = false;
            pickTriggered = false;
        }

        // 第二步：半抬
        if (passedNatural && currentArmState == ARM_STATE_HALF_RAISED) {
            passedHalf = true;
        }

        // 第三步：摘果子
        if (passedNatural && passedHalf && currentArmState == ARM_STATE_PICKING) {
            if (!pickTriggered && (now - stateHoldStart >= 200)) {
                pickTriggered = true;
                pickSignalStart = now;
                Serial.println(">>> PICK_FRUIT_ACTION DETECTED <<<");
                digitalWrite(13, HIGH);  // LED 反馈
            }
        }

        // LED 500ms 后关闭
        if (pickTriggered && (now - pickSignalStart >= 500)) {
            digitalWrite(13, LOW);
        }
    }

    // ===== 测试：打印数据 =====
    void test() {
        Serial.print("Roll: ");
        Serial.print(roll, 2);
        Serial.print(" Pitch: ");
        Serial.print(pitch, 2);
        Serial.print(" Yaw: ");
        Serial.println(yaw, 2);

        Serial.print("Ax: ");
        Serial.print(ax, 3);
        Serial.print(" Ay: ");
        Serial.print(ay, 3);
        Serial.print(" Az: ");
        Serial.println(az, 3);

        Serial.print("Gx: ");
        Serial.print(gx, 3);
        Serial.print(" Gy: ");
        Serial.print(gy, 3);
        Serial.print(" Gz: ");
        Serial.println(gz, 3);
    }

    // 打印手势状态
    void printArmState() {
        const char* stateNames[] = {"UNKNOWN", "NATURAL_DOWN", "HALF_RAISED", "PICKING"};
        Serial.print("ArmState: ");
        Serial.print(stateNames[currentArmState]);
        Serial.print(" | Seq: natural=");
        Serial.print(passedNatural ? "1" : "0");
        Serial.print(" half=");
        Serial.print(passedHalf ? "1" : "0");
        Serial.print(" pick=");
        Serial.println(pickTriggered ? "1" : "0");
    }

private:
    // 接收一个字节
    void receiveByte(uint8_t byte) {
        switch (rxState) {
            case RX_STATE_EXPECT_HEAD1:
                rxState = (byte == FRAME_HEAD1) ? RX_STATE_EXPECT_HEAD2 : RX_STATE_EXPECT_HEAD1;
                break;

            case RX_STATE_EXPECT_HEAD2:
                rxState = (byte == FRAME_HEAD2) ? RX_STATE_EXPECT_LENGTH : RX_STATE_EXPECT_HEAD1;
                break;

            case RX_STATE_EXPECT_LENGTH:
                frameLength = byte;
                rxState = RX_STATE_EXPECT_FUNCTION;
                break;

            case RX_STATE_EXPECT_FUNCTION:
                frameFunction = byte;
                frameIndex = 0;
                rxState = RX_STATE_COLLECT_DATA;
                break;

            case RX_STATE_COLLECT_DATA: {
                uint16_t dataLength = (frameLength >= 4) ? (uint16_t)(frameLength - 4) : 0;
                if (dataLength == 0 || dataLength > sizeof(frameBuffer)) {
                    rxState = RX_STATE_EXPECT_HEAD1;
                    break;
                }
                frameBuffer[frameIndex++] = byte;
                if (frameIndex >= dataLength) {
                    payloadLength = dataLength - 1;

                    // 校验和
                    uint8_t checksum = FRAME_HEAD1 + FRAME_HEAD2 + frameLength + frameFunction;
                    for (uint16_t i = 0; i < payloadLength; ++i) {
                        checksum += frameBuffer[i];
                    }

                    if (checksum == frameBuffer[dataLength - 1]) {
                        parseFrameData();
                    }
                    rxState = RX_STATE_EXPECT_HEAD1;
                }
            } break;

            default:
                rxState = RX_STATE_EXPECT_HEAD1;
                break;
        }
    }

    // 解析数据帧
    void parseFrameData() {
        if (frameFunction == IMU_FUNC_RAW_ACCEL) {
            if (payloadLength < 12) return;

            // 加速度（小端存储）
            int16_t ax_raw = (int16_t)(frameBuffer[1] << 8 | frameBuffer[0]);
            int16_t ay_raw = (int16_t)(frameBuffer[3] << 8 | frameBuffer[2]);
            int16_t az_raw = (int16_t)(frameBuffer[5] << 8 | frameBuffer[4]);

            ax = ax_raw * (16.0f / 32767.0f);
            ay = ay_raw * (16.0f / 32767.0f);
            az = az_raw * (16.0f / 32767.0f);

            // 角速度
            float gyroRatio = (2000.0f / 32767.0f) * (PI / 180.0f);
            gx = ((int16_t)(frameBuffer[7] << 8 | frameBuffer[6])) * gyroRatio;
            gy = ((int16_t)(frameBuffer[9] << 8 | frameBuffer[8])) * gyroRatio;
            gz = ((int16_t)(frameBuffer[11] << 8 | frameBuffer[10])) * gyroRatio;

            // Madgwick 滤波
            filter.update(gx, gy, gz, ax, ay, az, 0.0f, 0.0f, 0.0f);
            roll  = filter.getRoll();
            pitch = filter.getPitch();
            yaw   = filter.getYaw();
        }
        else if (frameFunction == IMU_FUNC_EULER) {
            if (payloadLength < 12) return;
            const float RAD2DEG = 57.2957795f;
            roll  = toFloat(&frameBuffer[0]) * RAD2DEG;
            pitch = toFloat(&frameBuffer[4]) * RAD2DEG;
            yaw   = toFloat(&frameBuffer[8]) * RAD2DEG;
        }
    }

    // 小端转 float
    float toFloat(const uint8_t *bytes) {
        float value;
        memcpy(&value, bytes, sizeof(float));
        return value;
    }

public:
    // ===== 获取数据 =====
    float getRoll() const { return roll; }
    float getPitch() const { return pitch; }
    float getYaw() const { return yaw; }

    float getAx() const { return ax; }
    float getAy() const { return ay; }
    float getAz() const { return az; }

    float getGx() const { return gx; }
    float getGy() const { return gy; }
    float getGz() const { return gz; }

    ArmState getArmState() const { return currentArmState; }
};

#endif