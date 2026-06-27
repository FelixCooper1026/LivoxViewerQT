#ifndef LIVOXVIEWER_SLAMUIBRIDGE_H
#define LIVOXVIEWER_SLAMUIBRIDGE_H

#include "Slam/Core/SlamTypes.h"
#include "Slam/Visualization/SlamRenderSnapshot.h"

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
    QString m_errorMessage;
    QTimer m_refreshTimer;
    QString m_mode = QStringLiteral("Idle");
    QString m_backend = QStringLiteral("FAST_LIO");
};

#endif // LIVOXVIEWER_SLAMUIBRIDGE_H
