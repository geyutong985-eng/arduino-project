// Pretest calibration sketch
// Records structured trial data for participant-specific DTW calibration.

#include <Arduino.h>
#include <Wire.h>
#include "../main/SensorIMU.h"

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

// ===== ESP32 pins =====
const int FLEX_PIN = 34;
const int PRESSURE_PIN = 35;
const int IMU_SDA_PIN = 21;
const int IMU_SCL_PIN = 22;

const uint8_t IMU1_ADDR = 0x68;
const uint8_t IMU2_ADDR = 0x69;

const unsigned long SAMPLE_DT_MS = 50;
const unsigned long BLOCK_CAPTURE_MS = 2000;

SensorIMU imuUpper(IMU1_ADDR, "IMU1");
SensorIMU imuForearm(IMU2_ADDR, "IMU2");

enum ActionId {
    ACTION_NONE = 0,
    ACTION_HALF_RAISED,
    ACTION_PICKING
};

const char* actionName(ActionId action) {
    switch (action) {
        case ACTION_HALF_RAISED:
            return "HALF_RAISED";
        case ACTION_PICKING:
            return "PICKING";
        default:
            return "NONE";
    }
}

class MadgwickIMU {
private:
    float beta;
    float q0;
    float q1;
    float q2;
    float q3;

    float invSqrt(float x) {
        return 1.0f / sqrtf(x);
    }

public:
    MadgwickIMU() : beta(0.12f), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

    void reset() {
        q0 = 1.0f;
        q1 = 0.0f;
        q2 = 0.0f;
        q3 = 0.0f;
    }

    void update(float gxDeg, float gyDeg, float gzDeg, float ax, float ay, float az, float dt) {
        float gx = gxDeg * DEG_TO_RAD;
        float gy = gyDeg * DEG_TO_RAD;
        float gz = gzDeg * DEG_TO_RAD;

        float norm = ax * ax + ay * ay + az * az;
        if (norm <= 0.0f || dt <= 0.0f) {
            return;
        }
        norm = invSqrt(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;

        float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
        float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
        float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

        float s0 = -2.0f * q2 * f1 + 2.0f * q1 * f2;
        float s1 =  2.0f * q3 * f1 + 2.0f * q0 * f2 - 4.0f * q1 * f3;
        float s2 = -2.0f * q0 * f1 + 2.0f * q3 * f2 - 4.0f * q2 * f3;
        float s3 =  2.0f * q1 * f1 + 2.0f * q2 * f2;

        norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (norm > 0.0f) {
            norm = invSqrt(norm);
            s0 *= norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
        }

        float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
        float qDot1 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy) - beta * s1;
        float qDot2 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx) - beta * s2;
        float qDot3 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx) - beta * s3;

        q0 += qDot0 * dt;
        q1 += qDot1 * dt;
        q2 += qDot2 * dt;
        q3 += qDot3 * dt;

        norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= norm;
        q1 *= norm;
        q2 *= norm;
        q3 *= norm;
    }

    void getQuaternion(float &w, float &x, float &y, float &z) const {
        w = q0;
        x = q1;
        y = q2;
        z = q3;
    }
};

MadgwickIMU upperFilter;
MadgwickIMU forearmFilter;
Quaternion upperNeutral = {1.0f, 0.0f, 0.0f, 0.0f};
Quaternion forearmNeutral = {1.0f, 0.0f, 0.0f, 0.0f};
bool neutralCaptured = false;

int flexStraightRaw = 0;
int flexBentRaw = 0;
bool flexStraightCaptured = false;
bool flexBentCaptured = false;

ActionId currentAction = ACTION_NONE;
bool trialActive = false;
int trialId = 0;
int lastEndedTrialId = 0;
ActionId lastEndedAction = ACTION_NONE;
unsigned long lastSampleTime = 0;
unsigned long lastFilterTime = 0;

Quaternion inverseQ(const Quaternion &q) {
    return {q.w, -q.x, -q.y, -q.z};
}

Quaternion multiplyQ(const Quaternion &a, const Quaternion &b) {
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    };
}

float relativeAngleDeg(const Quaternion &neutral, const Quaternion &current) {
    Quaternion rel = multiplyQ(inverseQ(neutral), current);
    float w = constrain(rel.w, -1.0f, 1.0f);
    float angle = 2.0f * acosf(w) * RAD_TO_DEG;
    if (angle > 180.0f) {
        angle = 360.0f - angle;
    }
    return angle;
}

int readAverageAnalog(int pin, int samples) {
    long sum = 0;
    for (int i = 0; i < samples; ++i) {
        sum += analogRead(pin);
        delay(4);
    }
    return (int)(sum / samples);
}

float getFlexAngle(int raw) {
    if (!flexStraightCaptured || !flexBentCaptured || flexStraightRaw == flexBentRaw) {
        return 0.0f;
    }
    float normalized = (float)(raw - flexStraightRaw) / (float)(flexBentRaw - flexStraightRaw);
    normalized = constrain(normalized, 0.0f, 1.0f);
    return normalized * 180.0f;
}

bool pressurePressed(int raw) {
    return raw >= 150;
}

void updateFilters() {
    unsigned long now = millis();
    float dt = (lastFilterTime == 0) ? (SAMPLE_DT_MS / 1000.0f) : ((now - lastFilterTime) / 1000.0f);
    lastFilterTime = now;

    imuUpper.update();
    imuForearm.update();

    upperFilter.update(
        imuUpper.getGyroX(), imuUpper.getGyroY(), imuUpper.getGyroZ(),
        imuUpper.getAccX(), imuUpper.getAccY(), imuUpper.getAccZ(),
        dt
    );
    forearmFilter.update(
        imuForearm.getGyroX(), imuForearm.getGyroY(), imuForearm.getGyroZ(),
        imuForearm.getAccX(), imuForearm.getAccY(), imuForearm.getAccZ(),
        dt
    );
}

void getCurrentQuaternions(Quaternion &upper, Quaternion &forearm) {
    upperFilter.getQuaternion(upper.w, upper.x, upper.y, upper.z);
    forearmFilter.getQuaternion(forearm.w, forearm.x, forearm.y, forearm.z);
}

void printDataLine(const char* phase, int dataTrialId, const char* dataAction) {
    Quaternion upperCurrent;
    Quaternion forearmCurrent;
    getCurrentQuaternions(upperCurrent, forearmCurrent);

    float upperAngle = neutralCaptured ? relativeAngleDeg(upperNeutral, upperCurrent) : 0.0f;
    float forearmAngle = neutralCaptured ? relativeAngleDeg(forearmNeutral, forearmCurrent) : 0.0f;

    int flexRaw = readAverageAnalog(FLEX_PIN, 5);
    float flexAngle = getFlexAngle(flexRaw);
    int pressureRaw = analogRead(PRESSURE_PIN);

    Serial.print(F("DATA,"));
    Serial.print(millis());
    Serial.print(',');
    Serial.print(phase);
    Serial.print(',');
    Serial.print(dataTrialId);
    Serial.print(',');
    Serial.print(dataAction);
    Serial.print(',');
    Serial.print(imuUpper.getAccX(), 3);
    Serial.print(',');
    Serial.print(imuUpper.getAccY(), 3);
    Serial.print(',');
    Serial.print(imuUpper.getAccZ(), 3);
    Serial.print(',');
    Serial.print(imuUpper.getGyroX(), 3);
    Serial.print(',');
    Serial.print(imuUpper.getGyroY(), 3);
    Serial.print(',');
    Serial.print(imuUpper.getGyroZ(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getAccX(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getAccY(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getAccZ(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getGyroX(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getGyroY(), 3);
    Serial.print(',');
    Serial.print(imuForearm.getGyroZ(), 3);
    Serial.print(',');
    Serial.print(upperAngle, 2);
    Serial.print(',');
    Serial.print(forearmAngle, 2);
    Serial.print(',');
    Serial.print(flexRaw);
    Serial.print(',');
    Serial.print(flexAngle, 1);
    Serial.print(',');
    Serial.print(pressureRaw);
    Serial.print(',');
    Serial.println(pressurePressed(pressureRaw) ? 1 : 0);
}

void captureNeutral() {
    Serial.println(F("NEUTRAL_START"));
    upperFilter.reset();
    forearmFilter.reset();
    lastFilterTime = 0;

    unsigned long start = millis();
    while (millis() - start < BLOCK_CAPTURE_MS) {
        updateFilters();
        printDataLine("NEUTRAL", 0, "NONE");
        delay(SAMPLE_DT_MS);
    }

    getCurrentQuaternions(upperNeutral, forearmNeutral);
    neutralCaptured = true;
    Serial.println(F("NEUTRAL_END"));
}

void captureFlexStraight() {
    Serial.println(F("FLEX_STRAIGHT_START"));
    unsigned long start = millis();
    while (millis() - start < BLOCK_CAPTURE_MS) {
        updateFilters();
        printDataLine("FLEX_STRAIGHT", 0, "NONE");
        delay(SAMPLE_DT_MS);
    }
    flexStraightRaw = readAverageAnalog(FLEX_PIN, 25);
    flexStraightCaptured = true;
    Serial.print(F("FLEX_STRAIGHT_END,"));
    Serial.println(flexStraightRaw);
}

void captureFlexBent() {
    Serial.println(F("FLEX_BENT_START"));
    unsigned long start = millis();
    while (millis() - start < BLOCK_CAPTURE_MS) {
        updateFilters();
        printDataLine("FLEX_BENT", 0, "NONE");
        delay(SAMPLE_DT_MS);
    }
    flexBentRaw = readAverageAnalog(FLEX_PIN, 25);
    flexBentCaptured = true;
    Serial.print(F("FLEX_BENT_END,"));
    Serial.println(flexBentRaw);
}

void startTrial() {
    if (currentAction == ACTION_NONE) {
        Serial.println(F("ERROR,SELECT_ACTION_FIRST"));
        return;
    }
    if (trialActive) {
        Serial.println(F("ERROR,TRIAL_ALREADY_ACTIVE"));
        return;
    }
    trialId++;
    trialActive = true;
    Serial.print(F("TRIAL_START,"));
    Serial.print(actionName(currentAction));
    Serial.print(',');
    Serial.println(trialId);
}

void endTrial() {
    if (!trialActive) {
        Serial.println(F("ERROR,NO_ACTIVE_TRIAL"));
        return;
    }
    trialActive = false;
    lastEndedTrialId = trialId;
    lastEndedAction = currentAction;
    Serial.print(F("TRIAL_END,"));
    Serial.print(actionName(currentAction));
    Serial.print(',');
    Serial.println(trialId);
}

void markTrial(bool accepted) {
    if (lastEndedAction == ACTION_NONE || lastEndedTrialId == 0) {
        Serial.println(F("ERROR,NO_ENDED_TRIAL_TO_MARK"));
        return;
    }
    Serial.print(accepted ? F("TRIAL_ACCEPT,") : F("TRIAL_REJECT,"));
    Serial.print(actionName(lastEndedAction));
    Serial.print(',');
    Serial.println(lastEndedTrialId);
}

void printHelp() {
    Serial.println(F("===== Pretest calibration commands ====="));
    Serial.println(F("n -> neutral pose capture"));
    Serial.println(F("f -> flex straight capture"));
    Serial.println(F("b -> flex bent capture"));
    Serial.println(F("1 -> select HALF_RAISED action"));
    Serial.println(F("2 -> select PICKING action"));
    Serial.println(F("s -> start trial"));
    Serial.println(F("e -> end trial"));
    Serial.println(F("a -> accept last ended trial"));
    Serial.println(F("r -> reject last ended trial"));
    Serial.println(F("h -> help"));
}

void handleCommand() {
    if (!Serial.available()) {
        return;
    }

    char cmd = Serial.read();
    if (cmd == '\n' || cmd == '\r') {
        return;
    }

    switch (cmd) {
        case 'n':
            captureNeutral();
            break;
        case 'f':
            captureFlexStraight();
            break;
        case 'b':
            captureFlexBent();
            break;
        case '1':
            currentAction = ACTION_HALF_RAISED;
            Serial.println(F("ACTION_SET,HALF_RAISED"));
            break;
        case '2':
            currentAction = ACTION_PICKING;
            Serial.println(F("ACTION_SET,PICKING"));
            break;
        case 's':
            startTrial();
            break;
        case 'e':
            endTrial();
            break;
        case 'a':
            markTrial(true);
            break;
        case 'r':
            markTrial(false);
            break;
        case 'h':
            printHelp();
            break;
        default:
            Serial.println(F("ERROR,UNKNOWN_COMMAND"));
            break;
    }
}

void setup() {
    Serial.begin(115200);
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(100000);
    pinMode(FLEX_PIN, INPUT);
    pinMode(PRESSURE_PIN, INPUT);
    delay(100);

    bool imu1Ok = imuUpper.begin();
    delay(10);
    bool imu2Ok = imuForearm.begin();
    delay(10);

    Serial.println(F("PRETEST_CALIBRATION_READY"));
    Serial.println(F("DATA_HEADER,timeMs,phase,trialId,action,imu1Ax,imu1Ay,imu1Az,imu1Gx,imu1Gy,imu1Gz,imu2Ax,imu2Ay,imu2Az,imu2Gx,imu2Gy,imu2Gz,upperAngle,forearmAngle,flexRaw,flexAngle,pressureRaw,pressurePressed"));
    Serial.print(F("IMU_STATUS,IMU1,"));
    Serial.println(imu1Ok ? F("OK") : F("FAIL"));
    Serial.print(F("IMU_STATUS,IMU2,"));
    Serial.println(imu2Ok ? F("OK") : F("FAIL"));
    printHelp();
}

void loop() {
    handleCommand();
    updateFilters();

    unsigned long now = millis();
    if (trialActive && now - lastSampleTime >= SAMPLE_DT_MS) {
        lastSampleTime = now;
        printDataLine("TRIAL", trialId, actionName(currentAction));
    }
}
