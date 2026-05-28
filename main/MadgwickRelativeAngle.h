#ifndef MADGWICK_RELATIVE_ANGLE_H
#define MADGWICK_RELATIVE_ANGLE_H

#include <Arduino.h>

struct DtwQuaternion {
    float w;
    float x;
    float y;
    float z;
};

class MadgwickRelativeAngle {
private:
    float beta;
    DtwQuaternion q;
    DtwQuaternion neutral;
    bool neutralSet;

    float invSqrt(float x) const {
        return 1.0f / sqrtf(x);
    }

    DtwQuaternion inverseQ(const DtwQuaternion &v) const {
        return {v.w, -v.x, -v.y, -v.z};
    }

    DtwQuaternion multiplyQ(const DtwQuaternion &a, const DtwQuaternion &b) const {
        return {
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
            a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        };
    }

public:
    MadgwickRelativeAngle()
        : beta(0.12f), q{1.0f, 0.0f, 0.0f, 0.0f},
          neutral{1.0f, 0.0f, 0.0f, 0.0f}, neutralSet(false) {}

    void reset() {
        q = {1.0f, 0.0f, 0.0f, 0.0f};
        neutralSet = false;
    }

    void update(float gxDeg, float gyDeg, float gzDeg,
                float ax, float ay, float az,
                float dtSeconds) {
        if (dtSeconds <= 0.0f) {
            return;
        }

        float gx = gxDeg * DEG_TO_RAD;
        float gy = gyDeg * DEG_TO_RAD;
        float gz = gzDeg * DEG_TO_RAD;

        float norm = ax * ax + ay * ay + az * az;
        if (norm <= 0.0f) {
            return;
        }
        norm = invSqrt(norm);
        ax *= norm;
        ay *= norm;
        az *= norm;

        float f1 = 2.0f * (q.x * q.z - q.w * q.y) - ax;
        float f2 = 2.0f * (q.w * q.x + q.y * q.z) - ay;
        float f3 = 2.0f * (0.5f - q.x * q.x - q.y * q.y) - az;

        float s0 = -2.0f * q.y * f1 + 2.0f * q.x * f2;
        float s1 =  2.0f * q.z * f1 + 2.0f * q.w * f2 - 4.0f * q.x * f3;
        float s2 = -2.0f * q.w * f1 + 2.0f * q.z * f2 - 4.0f * q.y * f3;
        float s3 =  2.0f * q.x * f1 + 2.0f * q.y * f2;

        norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
        if (norm > 0.0f) {
            norm = invSqrt(norm);
            s0 *= norm;
            s1 *= norm;
            s2 *= norm;
            s3 *= norm;
        }

        float qDot0 = 0.5f * (-q.x * gx - q.y * gy - q.z * gz) - beta * s0;
        float qDot1 = 0.5f * ( q.w * gx + q.y * gz - q.z * gy) - beta * s1;
        float qDot2 = 0.5f * ( q.w * gy - q.x * gz + q.z * gx) - beta * s2;
        float qDot3 = 0.5f * ( q.w * gz + q.x * gy - q.y * gx) - beta * s3;

        q.w += qDot0 * dtSeconds;
        q.x += qDot1 * dtSeconds;
        q.y += qDot2 * dtSeconds;
        q.z += qDot3 * dtSeconds;

        norm = invSqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        q.w *= norm;
        q.x *= norm;
        q.y *= norm;
        q.z *= norm;
    }

    void captureNeutral() {
        neutral = q;
        neutralSet = true;
    }

    bool hasNeutral() const {
        return neutralSet;
    }

    float getRelativeAngleDeg() const {
        if (!neutralSet) {
            return 0.0f;
        }

        DtwQuaternion rel = multiplyQ(inverseQ(neutral), q);
        float w = constrain(rel.w, -1.0f, 1.0f);
        float angle = 2.0f * acosf(w) * RAD_TO_DEG;
        if (angle > 180.0f) {
            angle = 360.0f - angle;
        }
        return angle;
    }
};

#endif
