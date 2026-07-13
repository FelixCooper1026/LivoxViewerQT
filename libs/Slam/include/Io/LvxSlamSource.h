#ifndef SLAM_IO_LVXSLAMSOURCE_H
#define SLAM_IO_LVXSLAMSOURCE_H

#include "Core/SlamTypes.h"

#include <QString>
#include <QVector>

class LvxSlamSource {
public:
    explicit LvxSlamSource(int frameDurationMs = 100);

    bool load(const QString& filePath, QString* error);
    const QVector<SlamInputFrame>& frames() const;
    QString summaryText() const;

private:
    QVector<SlamInputFrame> frames_;
    QString summaryText_;
    int64_t frameDurationNs_ = 100000000;
};

#endif // SLAM_IO_LVXSLAMSOURCE_H
