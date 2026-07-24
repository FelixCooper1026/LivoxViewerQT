#ifndef LIVOXVIEWER_SLAMUIBRIDGE_H
#define LIVOXVIEWER_SLAMUIBRIDGE_H

#include "Core/SlamTypes.h"
#include "Visualization/SlamRenderSnapshot.h"

#include <QColor>
#include <QImage>
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
        QString keyframeCount;
        QString loopClosureCount;
        QString memoryUsage;
        QString mapPoints;
        QString worldFramePoints;
        QString bodyFramePoints;
        QString globalMapPoints;
        QString dynamicMode;
        QString dynamicPoints;
        QString dynamicDetectorMs;
        QString dynamicClusterMs;
        QString freeDomMap;
        QString freeDomStages;
        QString error;
    };

    explicit SlamUiBridge(QObject* parent = nullptr);

    const SlamOutput& latestOutput() const { return m_latestOutput; }
    DisplayState displayState() const { return m_displayState; }
    QVector<SlamTrajectoryPoint> trajectorySnapshot() const { return m_trajectory; }
    QVector<SlamTrajectoryPoint> unoptimizedTrajectorySnapshot() const { return m_unoptimizedTrajectory; }
    QVector<SlamPoint> globalMapSnapshot() const { return m_globalMapPoints; }
    QVector<SlamPoint> freeDomStaticMapSnapshot() const { return m_freeDomStaticMapPoints; }
    QVector<SlamPoint> freeDomStaticVoxelSnapshot() const { return m_freeDomStaticVoxelPoints; }
    QImage freeDomDepthImage() const { return m_freeDomDepthImage; }
    QImage freeDomEnhancedDepthImage() const { return m_freeDomEnhancedDepthImage; }

public slots:
    void receiveSlamOutput(const SlamOutput& output);
    void setWorldFrameColor(const QColor& color);
    void setBodyFrameColor(const QColor& color);
    void setDynamicObjectColor(const QColor& color);
    void setDynamicAggressiveColor(const QColor& color);
    void setDynamicModerateColor(const QColor& color);
    void setDynamicConservativeColor(const QColor& color);
    void setFreeDomScanVoxelColor(const QColor& color);
    void setFreeDomDynamicVoxelColor(const QColor& color);
    void setFreeDomRaycastedVoxelColor(const QColor& color);
    void setFreeDomFreeVoxelColor(const QColor& color);
    void setFreeDomStaticVoxelColor(const QColor& color);
    void setFreeDomEnhancedColor(const QColor& color);
    void setTrajectoryColor(const QColor& color);
    void setWorldFramePointSize(float sizePx);
    void setBodyFramePointSize(float sizePx);
    void setDynamicObjectPointSize(float sizePx);
    void setFreeDomScanVoxelPointSize(float sizePx);
    void setFreeDomDynamicVoxelPointSize(float sizePx);
    void setFreeDomRaycastedVoxelPointSize(float sizePx);
    void setFreeDomFreeVoxelPointSize(float sizePx);
    void setFreeDomStaticVoxelPointSize(float sizePx);
    void setFreeDomEnhancedPointSize(float sizePx);
    void setTrajectoryLineWidth(float widthPx);
    void setPoseAxisLength(float lengthM);
    void setPoseAxisLineWidth(float widthPx);
    void setRenderLayerVisibility(bool trajectoryVisible,
                                  bool poseAxisVisible,
                                  bool worldFrameVisible,
                                  bool bodyFrameVisible,
                                  bool dynamicObjectVisible,
                                  bool freeDomScanVoxelVisible,
                                  bool freeDomDynamicVoxelVisible,
                                  bool freeDomRaycastedVoxelVisible,
                                  bool freeDomFreeVoxelVisible,
                                  bool freeDomStaticVoxelVisible,
                                  bool freeDomEnhancedVisible);
    void setModeAndBackend(const QString& mode, const QString& backend);
    void setErrorMessage(const QString& message);
    void clearErrorMessage();
    void resetCurrentPose();
    void clearDisplay();

signals:
    void statusTextReady(const QString& text);
    void displayStateChanged();
    void renderSnapshotReady(const SlamRenderSnapshot& snapshot);
    void renderPoseReady(const SlamRenderPose& pose);
    void poseAxisVerticesReady(const QVector<SlamRenderVertex>& vertices);

private:
    struct DenseGlobalMapSegment {
        int poseCorrectionEpoch = 0;
        SlamPose pose;
        QVector<SlamPoint> points;
    };

    void refreshStatus();
    SlamRenderSnapshot buildRenderSnapshot();
    QVector<SlamRenderVertex> buildPoseAxisVertices() const;
    void appendTrajectory(const SlamOutput& output);
    void appendGlobalMap(const SlamOutput& output);
    void correctDenseGlobalMap(const QVector<SlamTrajectoryPoint>& optimizedTrajectory);
    void rebuildDenseGlobalMapSnapshot();
    qsizetype denseGlobalMapPointCount() const;

    SlamOutput m_latestOutput;
    DisplayState m_displayState;
    QVector<SlamTrajectoryPoint> m_trajectory;
    QVector<SlamTrajectoryPoint> m_unoptimizedTrajectory;
    QVector<SlamPoint> m_globalMapPoints;
    QVector<SlamPoint> m_freeDomScanVoxelPoints;
    QVector<SlamPoint> m_freeDomDynamicVoxelPoints;
    QVector<SlamPoint> m_freeDomRaycastedVoxelPoints;
    QVector<SlamPoint> m_freeDomFreeVoxelPoints;
    QVector<SlamPoint> m_freeDomStaticVoxelPoints;
    QVector<SlamPoint> m_freeDomStaticMapPoints;
    QVector<SlamPoint> m_freeDomEnhancedPoints;
    QImage m_freeDomDepthImage;
    QImage m_freeDomEnhancedDepthImage;
    QVector<DenseGlobalMapSegment> m_denseGlobalMapSegments;
    QVector<SlamLoopClosureEdge> m_loopClosureEdges;
    quint64 m_worldFramePointTotal = 0;
    QString m_errorMessage;
    QTimer m_refreshTimer;
    QString m_mode = QStringLiteral("Idle");
    QString m_backend = QStringLiteral("FAST_LIO");
    QColor m_worldFrameColor = QColor(255, 255, 255);
    QColor m_bodyFrameColor = QColor(255, 140, 26);
    QColor m_dynamicObjectColor = QColor(239, 41, 41);
    QColor m_dynamicAggressiveColor = QColor(255, 48, 48);
    QColor m_dynamicModerateColor = QColor(255, 150, 35);
    QColor m_dynamicConservativeColor = QColor(202, 80, 255);
    QColor m_freeDomScanVoxelColor = QColor(0, 210, 255);
    QColor m_freeDomDynamicVoxelColor = QColor(255, 35, 210);
    QColor m_freeDomRaycastedVoxelColor = QColor(255, 220, 40);
    QColor m_freeDomFreeVoxelColor = QColor(60, 230, 100);
    QColor m_freeDomStaticVoxelColor = QColor(180, 180, 190);
    QColor m_freeDomEnhancedColor = QColor(70, 120, 255);
    QColor m_trajectoryColor = QColor(26, 191, 255);
    float m_worldFramePointSizePx = 2.0f;
    float m_bodyFramePointSizePx = 2.5f;
    float m_dynamicObjectPointSizePx = 4.0f;
    float m_freeDomScanVoxelPointSizePx = 4.0f;
    float m_freeDomDynamicVoxelPointSizePx = 4.0f;
    float m_freeDomRaycastedVoxelPointSizePx = 4.0f;
    float m_freeDomFreeVoxelPointSizePx = 4.0f;
    float m_freeDomStaticVoxelPointSizePx = 4.0f;
    float m_freeDomEnhancedPointSizePx = 4.0f;
    float m_trajectoryLineWidthPx = 2.0f;
    float m_poseAxisLengthM = 0.8f;
    float m_poseAxisLineWidthPx = 3.0f;
    bool m_trajectoryVisible = true;
    bool m_poseAxisVisible = true;
    bool m_worldFrameVisible = true;
    bool m_bodyFrameVisible = true;
    bool m_dynamicObjectVisible = true;
    bool m_freeDomScanVoxelVisible = true;
    bool m_freeDomDynamicVoxelVisible = true;
    bool m_freeDomRaycastedVoxelVisible = true;
    bool m_freeDomFreeVoxelVisible = true;
    bool m_freeDomStaticVoxelVisible = true;
    bool m_freeDomEnhancedVisible = true;
    SlamPose m_initialPose;
    bool m_hasInitialPose = false;
    bool m_hasCurrentPose = false;
    bool m_hasOptimizedTrajectory = false;
    bool m_hasOptimizedGlobalMap = false;
};

#endif // LIVOXVIEWER_SLAMUIBRIDGE_H
