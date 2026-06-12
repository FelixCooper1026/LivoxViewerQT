#ifndef LIVOXVIEWER_IMU_IMUATTITUDEESTIMATOR_H
#define LIVOXVIEWER_IMU_IMUATTITUDEESTIMATOR_H

#include <QQuaternion>

class ImuAttitudeEstimator
{
public:
    struct Result {
        QQuaternion orientation;
        double rollDeg = 0.0;
        double pitchDeg = 0.0;
        double yawDeg = 0.0;
    };

    void reset();
    Result update(double timestampSec,
                  double gyroXRadSec,
                  double gyroYRadSec,
                  double gyroZRadSec,
                  double accXG,
                  double accYG,
                  double accZG);
    Result current() const;

private:
    void initializeFromAcceleration(double accXG, double accYG, double accZG);
    void normalizeQuaternion();
    Result makeResult() const;

    bool m_initialized = false;
    double m_lastTimestampSec = 0.0;
    double m_q0 = 1.0;
    double m_q1 = 0.0;
    double m_q2 = 0.0;
    double m_q3 = 0.0;
    double m_beta = 0.08;
};

#endif // LIVOXVIEWER_IMU_IMUATTITUDEESTIMATOR_H
