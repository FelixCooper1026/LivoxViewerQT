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
        QString error;
    };

    explicit SlamUiBridge(QObject* parent = nullptr);

    const SlamOutput& latestOutput() const { return m_latestOutput; }
    DisplayState displayState() const { return m_displayState; }
    bool mapPreviewEnabled() const { return m_mapPreviewEnabled; }

public slots:
    void receiveSlamOutput(const SlamOutput& output);
    void setModeAndBackend(const QString& mode, const QString& backend);
    void setMapPreviewEnabled(bool enabled);
    void clearDisplay();

signals:
    void statusTextReady(const QString& text);
    void displayStateChanged();
    void renderSnapshotReady(const SlamRenderSnapshot& snapshot);

private:
    void refreshStatus();
    SlamRenderSnapshot buildRenderSnapshot();
    void appendTrajectory(const SlamOutput& output);
    void appendMapPreview(const SlamOutput& output);
    void flushPendingMapPreview();

    SlamOutput m_latestOutput;
    DisplayState m_displayState;
    QVector<SlamTrajectoryPoint> m_trajectory;
    QVector<SlamRenderVertex> m_mapPreviewPoints;
    QVector<SlamRenderVertex> m_pendingMapPreviewPoints;
    QTimer m_refreshTimer;
    QString m_mode = QStringLiteral("Idle");
    QString m_backend = QStringLiteral("FAST_LIO");
    bool m_mapPreviewEnabled = false;
    int m_maxMapPreviewPoints = 200000;
};

#endif // LIVOXVIEWER_SLAMUIBRIDGE_H
