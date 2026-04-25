#ifndef SENSOR_TF_H
#define SENSOR_TF_H

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class SensorTF {
private:
    uint8_t chipSelectPin;
    char logFileName[13];
    bool ready;

    bool openLogFile(File &file) {
        if (!ready) {
            return false;
        }

        file = SD.open(logFileName, FILE_WRITE);
        return (bool)file;
    }

public:
    SensorTF() : chipSelectPin(10), ready(false) {
        logFileName[0] = '\0';
    }

    bool begin(uint8_t csPin, const char *fileName = "imu_log.txt") {
        chipSelectPin = csPin;
        strncpy(logFileName, fileName, sizeof(logFileName) - 1);
        logFileName[sizeof(logFileName) - 1] = '\0';

        pinMode(chipSelectPin, OUTPUT);
        ready = SD.begin(chipSelectPin);
        if (!ready) {
            return false;
        }

        return ensureHeader();
    }

    bool isReady() const {
        return ready;
    }

    const char *getFileName() const {
        return logFileName;
    }

    bool ensureHeader() {
        File file;
        if (!openLogFile(file)) {
            ready = false;
            return false;
        }

        if (file.size() == 0) {
            file.println(F("ms,imu1_ax,imu1_ay,imu1_az,imu1_gx,imu1_gy,imu1_gz,imu1_posture,imu2_ax,imu2_ay,imu2_az,imu2_gx,imu2_gy,imu2_gz,imu2_posture,combined_posture,flex_raw,flex_angle,flex_state,pressure_raw,pressure_pressed,flex_pneumatic_enabled,vibration_enabled,is_inflating,inflate_triggered,vibration_active"));
        }

        file.close();
        return true;
    }

    bool appendEvent(unsigned long timestampMs, const __FlashStringHelper *eventText) {
        File file;
        if (!openLogFile(file)) {
            ready = false;
            return false;
        }

        file.print(F("#EVENT,"));
        file.print(timestampMs);
        file.print(',');
        file.println(eventText);
        file.close();
        return true;
    }

    bool appendEvent(unsigned long timestampMs, const char *eventText) {
        File file;
        if (!openLogFile(file)) {
            ready = false;
            return false;
        }

        file.print(F("#EVENT,"));
        file.print(timestampMs);
        file.print(',');
        file.println(eventText);
        file.close();
        return true;
    }

    bool appendSample(unsigned long timestampMs,
                      float imu1AccX,
                      float imu1AccY,
                      float imu1AccZ,
                      float imu1GyroX,
                      float imu1GyroY,
                      float imu1GyroZ,
                      int imu1Posture,
                      float imu2AccX,
                      float imu2AccY,
                      float imu2AccZ,
                      float imu2GyroX,
                      float imu2GyroY,
                      float imu2GyroZ,
                      int imu2Posture,
                      int combinedPosture,
                      int flexRaw,
                      float flexAngle,
                      int flexState,
                      int pressureRaw,
                      bool pressurePressed,
                      bool flexPneumaticEnabled,
                      bool vibrationEnabled,
                      bool isInflating,
                      bool inflateTriggered,
                      bool vibrationActive) {
        File file;
        if (!openLogFile(file)) {
            ready = false;
            return false;
        }

        file.print(timestampMs);
        file.print(',');
        file.print(imu1AccX, 3);
        file.print(',');
        file.print(imu1AccY, 3);
        file.print(',');
        file.print(imu1AccZ, 3);
        file.print(',');
        file.print(imu1GyroX, 3);
        file.print(',');
        file.print(imu1GyroY, 3);
        file.print(',');
        file.print(imu1GyroZ, 3);
        file.print(',');
        file.print(imu1Posture);
        file.print(',');
        file.print(imu2AccX, 3);
        file.print(',');
        file.print(imu2AccY, 3);
        file.print(',');
        file.print(imu2AccZ, 3);
        file.print(',');
        file.print(imu2GyroX, 3);
        file.print(',');
        file.print(imu2GyroY, 3);
        file.print(',');
        file.print(imu2GyroZ, 3);
        file.print(',');
        file.print(imu2Posture);
        file.print(',');
        file.print(combinedPosture);
        file.print(',');
        file.print(flexRaw);
        file.print(',');
        file.print(flexAngle, 2);
        file.print(',');
        file.print(flexState);
        file.print(',');
        file.print(pressureRaw);
        file.print(',');
        file.print(pressurePressed ? 1 : 0);
        file.print(',');
        file.print(flexPneumaticEnabled ? 1 : 0);
        file.print(',');
        file.print(vibrationEnabled ? 1 : 0);
        file.print(',');
        file.print(isInflating ? 1 : 0);
        file.print(',');
        file.print(inflateTriggered ? 1 : 0);
        file.print(',');
        file.println(vibrationActive ? 1 : 0);
        file.close();
        return true;
    }
};

#endif