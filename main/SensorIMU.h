#ifndef SENSORIMU_H
#define SENSORIMU_H

#include <Arduino.h>
#include <Wire.h>

#ifndef MPU6050_WHO_AM_I
#define MPU6050_WHO_AM_I 0x75
#endif

#ifndef MPU6050_PWR_MGMT_1
#define MPU6050_PWR_MGMT_1 0x6B
#endif

#ifndef MPU6050_ACCEL_CONFIG
#define MPU6050_ACCEL_CONFIG 0x1C
#endif

#ifndef MPU6050_GYRO_CONFIG
#define MPU6050_GYRO_CONFIG 0x1B
#endif

#ifndef MPU6050_CONFIG
#define MPU6050_CONFIG 0x1A
#endif

#ifndef MPU6050_ACCEL_XOUT_H
#define MPU6050_ACCEL_XOUT_H 0x3B
#endif

enum Posture {
    POSTURE_UNKNOWN = 0,
    POSTURE_NATURAL_DOWN,
    POSTURE_HALF_RAISED,
    POSTURE_PICKING
};

typedef Posture ArmState;
#define ARM_STATE_UNKNOWN       POSTURE_UNKNOWN
#define ARM_STATE_NATURAL_DOWN  POSTURE_NATURAL_DOWN
#define ARM_STATE_HALF_RAISED   POSTURE_HALF_RAISED
#define ARM_STATE_PICKING       POSTURE_PICKING

inline const char* postureToString(Posture posture) {
    switch (posture) {
        case POSTURE_NATURAL_DOWN:
            return "NATURAL_DOWN";
        case POSTURE_HALF_RAISED:
            return "HALF_RAISED";
        case POSTURE_PICKING:
            return "PICKING";
        default:
            return "UNKNOWN";
    }
}

class SensorIMU {
private:
    static bool wireStarted;

    uint8_t imuAddr;
    const char* imuName;
    bool online;

    int16_t accXRaw;
    int16_t accYRaw;
    int16_t accZRaw;
    int16_t gyroXRaw;
    int16_t gyroYRaw;
    int16_t gyroZRaw;
    int16_t tempRaw;

    Posture currentPosture;
    int stableCountThreshold;
    int naturalCount;
    int halfCount;
    int pickCount;

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

    bool isNatural(float ax, float ay, float az) {
        return (ax < -0.20f && ay < 0.45f && fabs(az) < 0.30f);
    }

    bool isHalfRaised(float ax, float ay, float az) {
        return (ax > 0.20f && ax < 0.75f && ay > 0.60f && az > 0.05f);
    }

    bool isPickPose(float ax, float ay, float az) {
        return (ax > 0.78f && ay > 0.15f && ay < 0.60f && fabs(az) < 0.20f);
    }

    void updatePostureState(float ax, float ay, float az) {
        bool natural = isNatural(ax, ay, az);
        bool half = isHalfRaised(ax, ay, az);
        bool pick = isPickPose(ax, ay, az);

        naturalCount = natural ? naturalCount + 1 : 0;
        halfCount = half ? halfCount + 1 : 0;
        pickCount = pick ? pickCount + 1 : 0;

        if (natural && naturalCount >= stableCountThreshold && currentPosture != POSTURE_NATURAL_DOWN) {
            currentPosture = POSTURE_NATURAL_DOWN;
            naturalCount = 0;
            Serial.print('[');
            Serial.print(imuName);
            Serial.println(F("] Posture: NATURAL_DOWN"));
        } else if (half && halfCount >= stableCountThreshold && currentPosture != POSTURE_HALF_RAISED) {
            currentPosture = POSTURE_HALF_RAISED;
            halfCount = 0;
            Serial.print('[');
            Serial.print(imuName);
            Serial.println(F("] Posture: HALF_RAISED"));
        } else if (pick && pickCount >= stableCountThreshold && currentPosture != POSTURE_PICKING) {
            currentPosture = POSTURE_PICKING;
            pickCount = 0;
            Serial.print('[');
            Serial.print(imuName);
            Serial.println(F("] Posture: PICKING"));
        }
    }

public:
    SensorIMU(uint8_t address = 0x68, const char* name = "IMU")
        : imuAddr(address), imuName(name), online(false),
          accXRaw(0), accYRaw(0), accZRaw(0),
          gyroXRaw(0), gyroYRaw(0), gyroZRaw(0), tempRaw(0),
          currentPosture(POSTURE_UNKNOWN), stableCountThreshold(2),
          naturalCount(0), halfCount(0), pickCount(0) {}

    bool begin() {
        if (!wireStarted) {
            Wire.begin();
            Wire.setClock(100000);
            delay(100);
            wireStarted = true;
        }

        writeRegister(MPU6050_PWR_MGMT_1, 0x00);
        delay(100);
        writeRegister(MPU6050_ACCEL_CONFIG, 0x00);
        writeRegister(MPU6050_GYRO_CONFIG, 0x00);
        writeRegister(MPU6050_CONFIG, 0x03);

        uint8_t whoAmI = 0;
        online = readRegisters(MPU6050_WHO_AM_I, 1, &whoAmI) && (whoAmI == imuAddr);
        return online;
    }

    bool read() {
        uint8_t buffer[14];
        if (!readRegisters(MPU6050_ACCEL_XOUT_H, 14, buffer)) {
            online = false;
            return false;
        }

        accXRaw = (int16_t)(buffer[0] << 8 | buffer[1]);
        accYRaw = (int16_t)(buffer[2] << 8 | buffer[3]);
        accZRaw = (int16_t)(buffer[4] << 8 | buffer[5]);
        tempRaw  = (int16_t)(buffer[6] << 8 | buffer[7]);
        gyroXRaw = (int16_t)(buffer[8] << 8 | buffer[9]);
        gyroYRaw = (int16_t)(buffer[10] << 8 | buffer[11]);
        gyroZRaw = (int16_t)(buffer[12] << 8 | buffer[13]);

        online = true;
        return true;
    }

    void update() {
        if (read()) {
            updatePostureState(getAccX(), getAccY(), getAccZ());
        }
    }

    float getAccX() const { return accXRaw / 16384.0f; }
    float getAccY() const { return accYRaw / 16384.0f; }
    float getAccZ() const { return accZRaw / 16384.0f; }

    float getGyroX() const { return gyroXRaw / 131.0f; }
    float getGyroY() const { return gyroYRaw / 131.0f; }
    float getGyroZ() const { return gyroZRaw / 131.0f; }

    float getTemperature() const { return tempRaw / 340.0f + 36.53f; }

    Posture getPosture() const { return currentPosture; }
    Posture getArmState() const { return currentPosture; }

    uint8_t getAddress() const { return imuAddr; }
    const char* getName() const { return imuName; }
    bool isOnline() const { return online; }

    void setStableCount(int count) { stableCountThreshold = count; }

    void test() {
        Serial.print(F("ACC: "));
        Serial.print(getAccX(), 3);
        Serial.print(F(", "));
        Serial.print(getAccY(), 3);
        Serial.print(F(", "));
        Serial.print(getAccZ(), 3);
    }

    void printLabeledData() {
        Serial.print('[');
        Serial.print(imuName);
        Serial.print(F("] ACC: "));
        Serial.print(getAccX(), 3);
        Serial.print(F(", "));
        Serial.print(getAccY(), 3);
        Serial.print(F(", "));
        Serial.print(getAccZ(), 3);
        Serial.print(F(" | GYRO: "));
        Serial.print(getGyroX(), 3);
        Serial.print(F(", "));
        Serial.print(getGyroY(), 3);
        Serial.print(F(", "));
        Serial.println(getGyroZ(), 3);
    }

    void printArmState() {
        Serial.print('[');
        Serial.print(imuName);
        Serial.print(F("] ArmState: "));
        Serial.println(postureToString(currentPosture));
    }
};

class DualIMUPostureClassifier {
private:
    struct PosePrototype {
        Posture posture;
        float imu1Ax;
        float imu1Ay;
        float imu1Az;
        float imu2Ax;
        float imu2Ay;
        float imu2Az;
    };

    static const PosePrototype prototypes[3];

    Posture currentPosture;
    Posture candidatePosture;
    uint8_t stableCount;
    uint8_t stableThreshold;
    float switchMargin;
    float currentScore;

    float squared(float value) const {
        return value * value;
    }

    float scorePose(const PosePrototype &pose,
                    const SensorIMU &imu1,
                    const SensorIMU &imu2) const {
        return squared(imu1.getAccX() - pose.imu1Ax) +
               squared(imu1.getAccY() - pose.imu1Ay) +
               squared(imu1.getAccZ() - pose.imu1Az) +
               squared(imu2.getAccX() - pose.imu2Ax) +
               squared(imu2.getAccY() - pose.imu2Ay) +
               squared(imu2.getAccZ() - pose.imu2Az);
    }

public:
    DualIMUPostureClassifier()
        : currentPosture(POSTURE_UNKNOWN),
          candidatePosture(POSTURE_UNKNOWN),
          stableCount(0),
          stableThreshold(3),
          switchMargin(0.08f),
          currentScore(999.0f) {}

    void setStableThreshold(uint8_t threshold) {
        stableThreshold = threshold;
    }

    void setSwitchMargin(float margin) {
        switchMargin = margin;
    }

    Posture update(const SensorIMU &imu1, const SensorIMU &imu2) {
        float bestScore = 999.0f;
        Posture bestPosture = POSTURE_UNKNOWN;

        for (uint8_t i = 0; i < 3; ++i) {
            float score = scorePose(prototypes[i], imu1, imu2);
            if (score < bestScore) {
                bestScore = score;
                bestPosture = prototypes[i].posture;
            }
        }

        // 分数太高说明匹配不好，返回UNKNOWN
        if (bestScore > 1.0f) {
            currentPosture = POSTURE_UNKNOWN;
            currentScore = bestScore;
            return currentPosture;
        }

        if (currentPosture == POSTURE_UNKNOWN) {
            currentPosture = bestPosture;
            candidatePosture = bestPosture;
            stableCount = 0;
            currentScore = bestScore;
            return currentPosture;
        }

        if (bestPosture == currentPosture) {
            candidatePosture = bestPosture;
            stableCount = 0;
            currentScore = bestScore;
            return currentPosture;
        }

        if (bestScore + switchMargin >= currentScore) {
            candidatePosture = currentPosture;
            stableCount = 0;
            return currentPosture;
        }

        if (candidatePosture != bestPosture) {
            candidatePosture = bestPosture;
            stableCount = 1;
            return currentPosture;
        }

        stableCount++;
        if (stableCount >= stableThreshold) {
            currentPosture = bestPosture;
            currentScore = bestScore;
            stableCount = 0;
        }

        return currentPosture;
    }

    Posture getPosture() const {
        return currentPosture;
    }

    float getCurrentScore() const {
        return currentScore;
    }
};

bool SensorIMU::wireStarted = false;

const DualIMUPostureClassifier::PosePrototype DualIMUPostureClassifier::prototypes[3] = {
    {POSTURE_NATURAL_DOWN, 0.133f, 0.189f, -0.996f, 0.903f, 0.247f, -0.513f},
    {POSTURE_HALF_RAISED,  0.781f, 0.323f,  0.557f, 0.220f, -0.797f, 0.596f},
    {POSTURE_PICKING,      0.914f, 0.304f, -0.344f, -0.730f, -0.608f, -0.120f}
};

#endif