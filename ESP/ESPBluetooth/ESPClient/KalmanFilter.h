// KalmanFilter.h

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include <math.h>

class KalmanFilter {
public:
    float processNoise;      // 프로세스 노이즈
    float measurementNoise;  // 측정 노이즈
    float estimatedError;    // 추정 오차
    float lastEstimate;      // 이전 추정 값

    KalmanFilter(float processNoise, float measurementNoise, float estimatedError, float initialEstimate) {
        this->processNoise = processNoise;
        this->measurementNoise = measurementNoise;
        this->estimatedError = estimatedError;
        this->lastEstimate = initialEstimate;
    }

    float updateEstimate(float measurement) {
        float kalmanGain = estimatedError / (estimatedError + measurementNoise);
        float currentEstimate = lastEstimate + kalmanGain * (measurement - lastEstimate);
        estimatedError = (1.0 - kalmanGain) * estimatedError + fabs(lastEstimate - currentEstimate) * processNoise;
        lastEstimate = currentEstimate;
        return currentEstimate;
    }
};

#endif // KALMANFILTER_H
