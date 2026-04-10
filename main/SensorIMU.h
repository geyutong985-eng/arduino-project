// IMU 姿态传感器模块（软件串口通信）
// 协议：帧头 0x7E 0x23，小端存储
// 软串口：D2(TX) D3(RX)

#ifndef SENSOR_IMU_H
#define SENSOR_IMU_H

#include <Arduino.h>
#include <SoftwareSerial.h>

// ===== 协议帧头和功能码 =====
#define FRAME_HEAD1 0x7E
#define FRAME_HEAD2 0x23
#define IMU_FUNC_RAW_ACCEL 0x04  // 原始数据
#define IMU_FUNC_QUAT       0x16  // 四元数
#define IMU_FUNC_EULER      0x26  // 欧拉角
#define IMU_FUNC_BARO       0x32  // 气压(10轴)

// ===== 解析状态 =====
enum {
    RX_STATE_EXPECT_HEAD1 = 0,
    RX_STATE_EXPECT_HEAD2,
    RX_STATE_EXPECT_LENGTH,
    RX_STATE_EXPECT_FUNCTION,
    RX_STATE_COLLECT_DATA
};

class SensorIMU {
private:
    SoftwareSerial imuSerial;  // 软串口 D2/D3

    // 解析状态机
    uint8_t rxState;
    uint8_t frameLength;
    uint8_t frameFunction;
    uint8_t frameBuffer[64];
    uint16_t frameIndex;

    // 解析后的数据
    float roll, pitch, yaw;        // 欧拉角(度)
    float ax, ay, az;             // 加速度(g)
    float gx, gy, gz;             // 角速度(rad/s)
    float q0, q1, q2, q3;         // 四元数

public:
    SensorIMU() : imuSerial(3, 2) {  // RX=D3, TX=D2
        reset();
    }

    // 初始化
    void init() {
        reset();
        imuSerial.begin(115200);
    }

    // 请求输出数据（初始化后调用）
    void requestEuler() {
        // 重置用户数据: 7E 23 07 A0 01 5F E8 (7字节)
        // 校验和 = 0x7E + 0x23 + 0x07 + 0xA0 + 0x01 + 0x5F = 0x1E8，取低8位 0xE8
        uint8_t cmdReset[] = {0x7E, 0x23, 0x07, 0xA0, 0x01, 0x5F, 0xE8};
        imuSerial.write(cmdReset, 7);
        delay(500);

        // 设置输出频率 50Hz: 7E 23 07 60 32 5F D9 (7字节)
        // 校验和 = 0x7E + 0x23 + 0x07 + 0x60 + 0x32 + 0x5F = 0x1D9，取低8位 0xD9
        uint8_t cmdFreq[] = {0x7E, 0x23, 0x07, 0x60, 0x32, 0x5F, 0xD9};  // 32=50Hz
        imuSerial.write(cmdFreq, 7);
        delay(100);
    }

    // 重置解析状态
    void reset() {
        rxState = RX_STATE_EXPECT_HEAD1;
        frameLength = 0;
        frameFunction = 0;
        frameIndex = 0;
    }

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
                    Serial.print("Bad length: ");
                    Serial.println(frameLength);
                    rxState = RX_STATE_EXPECT_HEAD1;
                    break;
                }
                frameBuffer[frameIndex++] = byte;
                if (frameIndex >= dataLength) {
                    // 校验
                    uint8_t checksum = FRAME_HEAD1 + FRAME_HEAD2 + frameLength + frameFunction;
                    for (uint16_t i = 0; i < dataLength - 1; ++i) {
                        checksum += frameBuffer[i];
                    }

                    Serial.print("Frame: len=");
                    Serial.print(frameLength, HEX);
                    Serial.print(" func=");
                    Serial.print(frameFunction, HEX);
                    Serial.print(" calc CS=");
                    Serial.print(checksum, HEX);
                    Serial.print(" recv CS=");
                    Serial.println(frameBuffer[dataLength - 1], HEX);

                    if (checksum == frameBuffer[dataLength - 1]) {
                        Serial.println("CHECK OK!");
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

    // 更新（主循环调用）
    void update() {
        if (imuSerial.available() > 0) {
            Serial.print("IMU RX: ");
            while (imuSerial.available()) {
                uint8_t b = imuSerial.read();
                Serial.print(b, HEX);
                Serial.print(" ");
            }
            Serial.println();
        }
        while (imuSerial.available()) {
            receiveByte(imuSerial.read());
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

    // 解析数据帧
    void parseFrameData() {
        Serial.print("Got frame func: ");
        Serial.println(frameFunction, HEX);

        if (frameFunction == IMU_FUNC_RAW_ACCEL) {
            // 加速度（小端存储，低字节在前）
            int16_t ax_raw = (int16_t)(frameBuffer[1] << 8 | frameBuffer[0]);
            int16_t ay_raw = (int16_t)(frameBuffer[3] << 8 | frameBuffer[2]);
            int16_t az_raw = (int16_t)(frameBuffer[5] << 8 | frameBuffer[4]);

            float accelRatio = 16.0f / 32767.0f;
            ax = ax_raw * accelRatio;
            ay = ay_raw * accelRatio;
            az = az_raw * accelRatio;

            // 角速度
            float gyroRatio = (2000.0f / 32767.0f) * (PI / 180.0f);
            gx = ((int16_t)(frameBuffer[7] << 8 | frameBuffer[6])) * gyroRatio;
            gy = ((int16_t)(frameBuffer[9] << 8 | frameBuffer[8])) * gyroRatio;
            gz = ((int16_t)(frameBuffer[11] << 8 | frameBuffer[10])) * gyroRatio;
        } else if (frameFunction == IMU_FUNC_EULER) {
            Serial.println("Parsing Euler...");
            // 欧拉角（弧度→度）
            const float RAD2DEG = 57.2957795f;
            roll = toFloat(&frameBuffer[0]) * RAD2DEG;
            pitch = toFloat(&frameBuffer[4]) * RAD2DEG;
            yaw = toFloat(&frameBuffer[8]) * RAD2DEG;
            Serial.print("Roll: ");
            Serial.println(roll);
        } else if (frameFunction == IMU_FUNC_QUAT) {
            // 四元数
            q0 = toFloat(&frameBuffer[0]);
            q1 = toFloat(&frameBuffer[4]);
            q2 = toFloat(&frameBuffer[8]);
            q3 = toFloat(&frameBuffer[12]);
        }
    }

    // ===== 获取数据 =====
    float getRoll() { return roll; }
    float getPitch() { return pitch; }
    float getYaw() { return yaw; }

    float getAx() { return ax; }
    float getAy() { return ay; }
    float getAz() { return az; }

    float getGx() { return gx; }
    float getGy() { return gy; }
    float getGz() { return gz; }

private:
    // 小端转 int16
    int16_t toInt16(const uint8_t *bytes) {
        return (int16_t)((bytes[1] << 8) + bytes[0]);
    }

    // 小端转 float
    float toFloat(const uint8_t *bytes) {
        float value;
        memcpy(&value, bytes, sizeof(float));
        return value;
    }
};

#endif
