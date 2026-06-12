#include "imu/ImuAttitudeEstimator.h"

#include <QtMath>
#include <QVector3D>

#include <algorithm>
#include <cmath>

void ImuAttitudeEstimator::reset()
{
    m_initialized = false;
    m_lastTimestampSec = 0.0;
    m_q0 = 1.0;
    m_q1 = 0.0;
    m_q2 = 0.0;
    m_q3 = 0.0;
}

ImuAttitudeEstimator::Result ImuAttitudeEstimator::update(double timestampSec,
                                                          double gyroXRadSec,
                                                          double gyroYRadSec,
                                                          double gyroZRadSec,
                                                          double accXG,
                                                          double accYG,
                                                          double accZG)
{
    if (!m_initialized) {
        initializeFromAcceleration(accXG, accYG, accZG);
        m_lastTimestampSec = timestampSec;
        return makeResult();
    }

    const double dt = timestampSec - m_lastTimestampSec;
    m_lastTimestampSec = timestampSec;
    if (dt <= 0.0) {
        return makeResult();
    }

    double qDot0 = 0.5 * (-m_q1 * gyroXRadSec - m_q2 * gyroYRadSec - m_q3 * gyroZRadSec);
    double qDot1 = 0.5 * ( m_q0 * gyroXRadSec + m_q2 * gyroZRadSec - m_q3 * gyroYRadSec);
    double qDot2 = 0.5 * ( m_q0 * gyroYRadSec - m_q1 * gyroZRadSec + m_q3 * gyroXRadSec);
    double qDot3 = 0.5 * ( m_q0 * gyroZRadSec + m_q1 * gyroYRadSec - m_q2 * gyroXRadSec);

    const double accNorm = std::sqrt(accXG * accXG + accYG * accYG + accZG * accZG);
    if (accNorm > 0.0) {
        const double ax = accXG / accNorm;
        const double ay = accYG / accNorm;
        const double az = accZG / accNorm;

        const double twoQ0 = 2.0 * m_q0;
        const double twoQ1 = 2.0 * m_q1;
        const double twoQ2 = 2.0 * m_q2;
        const double twoQ3 = 2.0 * m_q3;
        const double fourQ0 = 4.0 * m_q0;
        const double fourQ1 = 4.0 * m_q1;
        const double fourQ2 = 4.0 * m_q2;
        const double eightQ1 = 8.0 * m_q1;
        const double eightQ2 = 8.0 * m_q2;
        const double q0q0 = m_q0 * m_q0;
        const double q1q1 = m_q1 * m_q1;
        const double q2q2 = m_q2 * m_q2;
        const double q3q3 = m_q3 * m_q3;

        double s0 = fourQ0 * q2q2 + twoQ2 * ax + fourQ0 * q1q1 - twoQ1 * ay;
        double s1 = fourQ1 * q3q3 - twoQ3 * ax + 4.0 * q0q0 * m_q1 - twoQ0 * ay
                  - fourQ1 + eightQ1 * q1q1 + eightQ1 * q2q2 + fourQ1 * az;
        double s2 = 4.0 * q0q0 * m_q2 + twoQ0 * ax + fourQ2 * q3q3 - twoQ3 * ay
                  - fourQ2 + eightQ2 * q1q1 + eightQ2 * q2q2 + fourQ2 * az;
        double s3 = 4.0 * q1q1 * m_q3 - twoQ1 * ax + 4.0 * q2q2 * m_q3 - twoQ2 * ay;
        const double gradientNorm = std::sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        if (gradientNorm > 0.0) {
            s0 /= gradientNorm;
            s1 /= gradientNorm;
            s2 /= gradientNorm;
            s3 /= gradientNorm;
            qDot0 -= m_beta * s0;
            qDot1 -= m_beta * s1;
            qDot2 -= m_beta * s2;
            qDot3 -= m_beta * s3;
        }
    }

    m_q0 += qDot0 * dt;
    m_q1 += qDot1 * dt;
    m_q2 += qDot2 * dt;
    m_q3 += qDot3 * dt;
    normalizeQuaternion();

    return makeResult();
}

ImuAttitudeEstimator::Result ImuAttitudeEstimator::current() const
{
    return makeResult();
}

void ImuAttitudeEstimator::initializeFromAcceleration(double accXG, double accYG, double accZG)
{
    const double roll = std::atan2(accYG, accZG);
    const double pitch = std::atan2(-accXG, std::sqrt(accYG * accYG + accZG * accZG));
    const QQuaternion qRoll = QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), float(qRadiansToDegrees(roll)));
    const QQuaternion qPitch = QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), float(qRadiansToDegrees(pitch)));
    const QQuaternion q = qPitch * qRoll;
    m_q0 = q.scalar();
    m_q1 = q.x();
    m_q2 = q.y();
    m_q3 = q.z();
    normalizeQuaternion();
    m_initialized = true;
}

void ImuAttitudeEstimator::normalizeQuaternion()
{
    const double norm = std::sqrt(m_q0 * m_q0 + m_q1 * m_q1 + m_q2 * m_q2 + m_q3 * m_q3);
    if (norm > 0.0) {
        m_q0 /= norm;
        m_q1 /= norm;
        m_q2 /= norm;
        m_q3 /= norm;
    }
}

ImuAttitudeEstimator::Result ImuAttitudeEstimator::makeResult() const
{
    Result result;

    const double sinRollCosPitch = 2.0 * (m_q0 * m_q1 + m_q2 * m_q3);
    const double cosRollCosPitch = 1.0 - 2.0 * (m_q1 * m_q1 + m_q2 * m_q2);
    const double roll = std::atan2(sinRollCosPitch, cosRollCosPitch);

    const double sinPitch = 2.0 * (m_q0 * m_q2 - m_q3 * m_q1);
    constexpr double kHalfPi = 1.57079632679489661923;
    const double pitch = std::abs(sinPitch) >= 1.0
        ? std::copysign(kHalfPi, sinPitch)
        : std::asin(sinPitch);

    const double sinYawCosPitch = 2.0 * (m_q0 * m_q3 + m_q1 * m_q2);
    const double cosYawCosPitch = 1.0 - 2.0 * (m_q2 * m_q2 + m_q3 * m_q3);
    const double yaw = std::atan2(sinYawCosPitch, cosYawCosPitch);

    result.rollDeg = -qRadiansToDegrees(roll);
    result.pitchDeg = -qRadiansToDegrees(pitch);
    result.yawDeg = qRadiansToDegrees(yaw);

    const QQuaternion qRoll = QQuaternion::fromAxisAndAngle(QVector3D(1.0f, 0.0f, 0.0f), float(result.rollDeg));
    const QQuaternion qPitch = QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 1.0f, 0.0f), float(result.pitchDeg));
    const QQuaternion qYaw = QQuaternion::fromAxisAndAngle(QVector3D(0.0f, 0.0f, 1.0f), float(result.yawDeg));
    result.orientation = qYaw * qPitch * qRoll;
    return result;
}
