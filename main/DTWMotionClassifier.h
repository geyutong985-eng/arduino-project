#ifndef DTW_MOTION_CLASSIFIER_H
#define DTW_MOTION_CLASSIFIER_H

#include <Arduino.h>
#include "DTWCalibrationData.h"

struct DtwMotionResult {
    bool matched;
    uint8_t actionId;
    float distance;
    float threshold;
};

class DTWMotionClassifier {
private:
    static const uint8_t MAX_CAPTURE_FRAMES = 60;
    float capture[MAX_CAPTURE_FRAMES][DTW_FEATURE_COUNT];
    uint8_t captureCount;

    float frameDistance(const float *a, const float *b) const {
        float sum = 0.0f;
        for (uint8_t i = 0; i < DTW_FEATURE_COUNT; ++i) {
            float diff = a[i] - b[i];
            sum += diff * diff;
        }
        return sum;
    }

    uint8_t mapIndex(uint8_t index, uint8_t sourceCount, uint8_t targetCount) const {
        if (targetCount <= 1 || sourceCount <= 1) {
            return 0;
        }
        return (uint8_t)((uint16_t)index * (sourceCount - 1) / (targetCount - 1));
    }

    void buildDownsampled(float out[DTW_MAX_TEMPLATE_FRAMES][DTW_FEATURE_COUNT],
                          uint8_t &outCount) const {
        outCount = captureCount;
        if (outCount > DTW_MAX_TEMPLATE_FRAMES) {
            outCount = DTW_MAX_TEMPLATE_FRAMES;
        }

        for (uint8_t i = 0; i < outCount; ++i) {
            uint8_t src = mapIndex(i, captureCount, outCount);
            for (uint8_t f = 0; f < DTW_FEATURE_COUNT; ++f) {
                out[i][f] = capture[src][f];
            }
        }
    }

    float dtwDistance(const float sample[DTW_MAX_TEMPLATE_FRAMES][DTW_FEATURE_COUNT],
                      uint8_t sampleCount,
                      const DtwTemplateDef &templ) const {
        float prevCost[DTW_MAX_TEMPLATE_FRAMES + 1];
        float currCost[DTW_MAX_TEMPLATE_FRAMES + 1];
        uint8_t prevLen[DTW_MAX_TEMPLATE_FRAMES + 1];
        uint8_t currLen[DTW_MAX_TEMPLATE_FRAMES + 1];

        const float inf = 1000000.0f;
        for (uint8_t j = 0; j <= templ.frameCount; ++j) {
            prevCost[j] = inf;
            currCost[j] = inf;
            prevLen[j] = 0;
            currLen[j] = 0;
        }
        prevCost[0] = 0.0f;

        for (uint8_t i = 1; i <= sampleCount; ++i) {
            currCost[0] = inf;
            currLen[0] = 0;

            for (uint8_t j = 1; j <= templ.frameCount; ++j) {
                float diag = prevCost[j - 1];
                float up = prevCost[j];
                float left = currCost[j - 1];

                float best = diag;
                uint8_t bestLen = prevLen[j - 1];
                if (up < best) {
                    best = up;
                    bestLen = prevLen[j];
                }
                if (left < best) {
                    best = left;
                    bestLen = currLen[j - 1];
                }

                const float *templateFrame = &templ.frames[(j - 1) * DTW_FEATURE_COUNT];
                float local = frameDistance(sample[i - 1], templateFrame);
                currCost[j] = local + best;
                currLen[j] = bestLen + 1;
            }

            for (uint8_t j = 0; j <= templ.frameCount; ++j) {
                prevCost[j] = currCost[j];
                prevLen[j] = currLen[j];
            }
        }

        return prevCost[templ.frameCount] / max((uint8_t)1, prevLen[templ.frameCount]);
    }

public:
    DTWMotionClassifier() : captureCount(0) {}

    void reset() {
        captureCount = 0;
    }

    void addSample(const float features[DTW_FEATURE_COUNT]) {
        if (captureCount >= MAX_CAPTURE_FRAMES) {
            for (uint8_t i = 1; i < MAX_CAPTURE_FRAMES; ++i) {
                for (uint8_t f = 0; f < DTW_FEATURE_COUNT; ++f) {
                    capture[i - 1][f] = capture[i][f];
                }
            }
            captureCount = MAX_CAPTURE_FRAMES - 1;
        }

        for (uint8_t f = 0; f < DTW_FEATURE_COUNT; ++f) {
            capture[captureCount][f] = features[f];
        }
        captureCount++;
    }

    uint8_t getCaptureCount() const {
        return captureCount;
    }

    DtwMotionResult evaluate(uint8_t actionId) const {
        DtwMotionResult result = {false, actionId, 999999.0f, 0.0f};
        if (captureCount < 2) {
            return result;
        }

        float sample[DTW_MAX_TEMPLATE_FRAMES][DTW_FEATURE_COUNT];
        uint8_t sampleCount = 0;
        buildDownsampled(sample, sampleCount);

        for (uint8_t i = 0; i < DTW_TEMPLATE_COUNT; ++i) {
            const DtwTemplateDef &templ = DTW_TEMPLATES[i];
            if (templ.actionId != actionId || templ.frameCount == 0) {
                continue;
            }

            float distance = dtwDistance(sample, sampleCount, templ);
            if (distance < result.distance) {
                result.distance = distance;
                result.threshold = templ.threshold;
            }
        }

        result.matched = result.threshold > 0.0f && result.distance <= result.threshold;
        return result;
    }
};

#endif
