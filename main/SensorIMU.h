#ifndef SENSORIMU_H
#define SENSORIMU_H

#include <Arduino.h>
#include <Wire.h>

// ===== 手势识别状态 =====
enum Posture {
    POSTURE_UNKNOWN = 0,
    POSTURE_NATURAL_DOWN,  // 自然下垂
    POSTURE_HALF_RAISED,   // 半抬
    POSTURE_PICKING        // 摘果子/举起
};

// 兼容旧接口
typedef Posture ArmState;
#define ARM_STATE_UNKNOWN       POSTURE_UNKNOWN
#define ARM_STATE_NATURAL_DOWN  POSTURE_NATURAL_DOWN
#define ARM_STATE_HALF_RAISED   POSTURE_HALF_RAISED
#define ARM_STATE_PICKING       POSTURE_PICKING

class SensorIMU {
private:
    uint8_t imuAddr;

    // 原始数据
    int16_t accXRaw, accYRaw, accZRaw;
    int16_t gyroXRaw, gyroYRaw, gyroZRaw;
    int16_t tempRaw;

    // 手势识别状态
    Posture currentPosture;
    int stableCountThreshold;
    int naturalCount;
    int halfCount;
    int pickCount;

    // I2C 通信
    void writeRegister(uint8_t reg, uint8_t data) {
        Wire.beginTransmission(imuAddr);
        Wire.write(reg);
        Wire.write(data);
        Wire.endTransmission();
    }

    bool readRegisters(uint8_t startReg, uint8_t count, uint8_t *dest) {
        Wire.beginTransmission(imuAddr);
        Wire.write(startReg);
        if (Wire.endTransmission(false) != 0) {
            return false;
        }
        uint8_t received = Wire.requestFrom(imuAddr, count);
        if (received != count) {
            return false;
        }
        for (uint8_t i = 0; i < count; i++) {
            dest[i] = Wire.read();
        }
        return true;
    }

    // ===== 姿态判断函数 =====
    bool isNatural(float ax, float ay, float az) {
        return (ax < -0.20 && ay < 0.45 && abs(az) < 0.30);
    }

    bool isHalfRaised(float ax, float ay, float az) {
        return (ax > 0.20 && ax < 0.75 && ay > 0.60 && az > 0.05);
    }

    bool isPickPose(float ax, float ay, float az) {
        return (ax > 0.78 && ay > 0.15 && ay < 0.60 && abs(az) < 0.20);
    }

    // 更新姿态状态（自由切换）
    void updatePostureState(float ax, float ay, float az) {
        bool natural = isNatural(ax, ay, az);
        bool half = isHalfRaised(ax, ay, az);
        bool pick = isPickPose(ax, ay, az);

        if (natural) naturalCount++;
        else naturalCount = 0;

        if (half) halfCount++;
        else halfCount = 0;

        if (pick) pickCount++;
        else pickCount = 0;

        // 自由切换：检测到哪个状态且满足稳定次数就切换
        if (natural && naturalCount >= stableCountThreshold && currentPosture != POSTURE_NATURAL_DOWN) {
            currentPosture = POSTURE_NATURAL_DOWN;
            Serial.println("Posture: NATURAL DOWN");
            naturalCount = 0;
        } else if (half && halfCount >= stableCountThreshold && currentPosture != POSTURE_HALF_RAISED) {
            currentPosture = POSTURE_HALF_RAISED;
            Serial.println("Posture: HALF RAISED");
            halfCount = 0;
        } else if (pick && pickCount >= stableCountThreshold && currentPosture != POSTURE_PICKING) {
            currentPosture = POSTURE_PICKING;
            Serial.println("Posture: PICKING");
            pickCount = 0;
        }
    }

public:
    SensorIMU(uint8_t address = 0x68) {
        imuAddr = address;
        accXRaw = accYRaw = accZRaw = 0;
        gyroXRaw = gyroYRaw = gyroZRaw = 0;
        tempRaw = 0;
        currentPosture = POSTURE_UNKNOWN;
        stableCountThreshold = 2;
        naturalCount = halfCount = pickCount = 0;
    }

    // 初始化（替代 init()）
    bool begin() {
        Wire.begin();
        Wire.setClock(100000);
        delay(100);

        // 唤醒 MPU6050
        writeRegister(0x6B, 0x00);
        delay(100);

        // 加速度量程 ±2g
        writeRegister(0x1C, 0x00);
        // 陀螺仪量程 ±250°/s
        writeRegister(0x1B, 0x00);
        // 低通滤波
        writeRegister(0x1A, 0x03);

        // 检测是否能读到数据
        uint8_t testValue = 0;
        return readRegisters(0x75, 1, &testValue);
    }

    // 读取数据
    bool read() {
        uint8_t buffer[14];
        if (!readRegisters(0x3B, 14, buffer)) {
            return false;
        }

        accXRaw  = (int16_t)(buffer[0] << 8 | buffer[1]);
        accYRaw  = (int16_t)(buffer[2] << 8 | buffer[3]);
        accZRaw  = (int16_t)(buffer[4] << 8 | buffer[5]);
        tempRaw  = (int16_t)(buffer[6] << 8 | buffer[7]);
        gyroXRaw = (int16_t)(buffer[8] << 8 | buffer[9]);
        gyroYRaw = (int16_t)(buffer[10] << 8 | buffer[11]);
        gyroZRaw = (int16_t)(buffer[12] << 8 | buffer[13]);

        return true;
    }

    // 更新（主循环调用）
    void update() {
        if (read()) {
            updatePostureState(getAccX(), getAccY(), getAccZ());
        }
    }

    // ===== 获取加速度(g) =====
    float getAccX() { return accXRaw / 16384.0; }
    float getAccY() { return accYRaw / 16384.0; }
    float getAccZ() { return accZRaw / 16384.0; }

    // ===== 获取陀螺仪(deg/s) =====
    float getGyroX() { return gyroXRaw / 131.0; }
    float getGyroY() { return gyroYRaw / 131.0; }
    float getGyroZ() { return gyroZRaw / 131.0; }

    // ===== 获取温度 =====
    float getTemperature() { return tempRaw / 340.0 + 36.53; }

    // ===== 获取姿态状态 =====
    Posture getPosture() { return currentPosture; }

    // 兼容旧接口：获取 ArmState（项目中其他地方用）
    Posture getArmState() { return currentPosture; }

    // 设置稳定阈值
    void setStableCount(int count) { stableCountThreshold = count; }

    // ===== 测试打印 =====
    void test() {
        Serial.print("ACC: ");
        Serial.print(getAccX(), 3); Serial.print(", ");
        Serial.print(getAccY(), 3); Serial.print(", ");
        Serial.print(getAccZ(), 3);
    }

    void printArmState() {
        const char* names[] = {"UNKNOWN", "NATURAL_DOWN", "HALF_RAISED", "PICKING"};
        Serial.print("ArmState: ");
        Serial.println(names[currentPosture]);
    }
};

#endif