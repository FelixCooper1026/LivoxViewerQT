#ifndef LIVOXVIEWER_SLAMUIBRIDGE_H
#define LIVOXVIEWER_SLAMUIBRIDGE_H

#include "Core/SlamTypes.h"
#include "Visualization/SlamRenderSnapshot.h"

#include <QColor>
#include <QObject>
#include <QTimer>

class SlamUiBridge : public QObject
{
    Q_OBJECT

public:
    struct DisplayState {
        QString status;
        QString mode;
        QString backend;
        QString imuState;
        QString inputFps;
        QString backendMs;
        QString droppedFrames;
        QString currentPose;
        QString trajectoryPoints;
        QString memoryUsage;
        QString mapPoints;
        QString worldFramePoints;
        QString bodyFramePoints;
        QString globalMapPoints;
        QString error;
    };

    explicit SlamUiBridge(QObject* parent = nullptr);

    const SlamOutput& latestOutput() const { return m_latestOutput; }
    DisplayState displayState() const { return m_displayState; }
    QVector<SlamTrajectoryPoint> trajectorySnapshot() const { return m_trajectory; }
    QVector<SlamPoint> globalMapSnapshot() const { return m_globalMapPoints; }

public slots:
    void receiveSlamOutput(const SlamOutput& output);
    void setWorldFrameColor(const QColor& color);
    void setBodyFrameColor(const QColor& color);
    void setTrajectoryColor(const QColor& color);
    void setWorldFramePointSize(float sizePx);
    void setBodyFramePointSize(float sizePx);
    void setTrajectoryLineWidth(float widthPx);
    void setPoseAxisLength(float lengthM);
    void setPoseAxisLineWidth(float widthPx);
    void setRenderLayerVisibility(bool trajectoryVisible,
                                  bool poseAxisVisible,
                                  bool worldFrameVisible,
                                  bool bodyFrameVisible);
    void setModeAndBackend(const QString& mode, const QString& backend);
    void setErrorMessage(const QString& message);
    void clearErrorMessage();
    void clearDisplay();

signals:
    void statusTextReady(const QString& text);
    void displayStateChanged();
    void renderSnapshotReady(const SlamRenderSnapshot& snapshot);

private:
    void refreshStatus();
    SlamRenderSnapshot buildRenderSnapshot();
    void appendTrajectory(const SlamOutput& output);
    void appendGlobalMap(const SlamOutput& output);

    SlamOutput m_latestOutput;
    DisplayState m_displayState;
    QVector<SlamTrajectoryPoint> m_trajectory;
    QVector<SlamPoint> m_globalMapPoints;
    quint64 m_worldFramePointTotal = 0;
    QString m_errorMessage;
    QTimer m_refreshTimer;
    QString m_mode = QStringLiteral("Idle");
    QString m_backend = QStringLiteral("FAST_LIO");
    QColor m_worldFrameColor = QColor(255, 255, 255);
    QColor m_bodyFrameColor = QColor(255, 140, 26);
    QColor m_trajectoryColor = QColor(26, 191, 255);
    float m_worldFramePointSizePx = 2.0f;
    float m_bodyFramePointSizePx = 2.5f;
    float m_trajectoryLineWidthPx = 2.0f;
    float m_poseAxisLengthM = 0.8f;
    float m_poseAxisLineWidthPx = 3.0f;
    bool m_trajectoryVisible = true;
    bool m_poseAxisVisible = true;
    bool m_worldFrameVisible = true;
    bool m_bodyFrameVisible = true;
};

#endif // LIVOXVIEWER_SLAMUIBRIDGE_H
