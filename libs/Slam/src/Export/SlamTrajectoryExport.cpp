#include "Export/SlamTrajectoryExport.h"

#include <QFile>
#include <QTextStream>

namespace SlamTrajectoryExport {

namespace {

constexpr double kNsToSeconds = 1.0e-9;

void assignError(QString* error, const QString& message)
{
    if (error) {
        *error = message;
    }
}

bool openTextFile(QFile& file, QString* error)
{
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        assignError(error, QStringLiteral("无法创建轨迹文件: %1").arg(file.errorString()));
        return false;
    }
    return true;
}

} // namespace

bool save(const QString& filePath,
          const QVector<SlamTrajectoryPoint>& trajectory,
          Format format,
          QString* error)
{
    switch (format) {
    case Format::Csv:
        return saveCsv(filePath, trajectory, error);
    case Format::Tum:
        return saveTum(filePath, trajectory, error);
    }
    assignError(error, QStringLiteral("未知轨迹导出格式。"));
    return false;
}

bool saveCsv(const QString& filePath,
             const QVector<SlamTrajectoryPoint>& trajectory,
             QString* error)
{
    if (trajectory.isEmpty()) {
        assignError(error, QStringLiteral("当前没有可导出的 SLAM 轨迹。"));
        return false;
    }

    QFile file(filePath);
    if (!openTextFile(file, error)) {
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(9);
    out << "timestamp_ns,timestamp_s,tx,ty,tz,qx,qy,qz,qw,quality,pose_frame\n";
    for (const SlamTrajectoryPoint& point : trajectory) {
        const SlamPose& pose = point.pose;
        out << pose.timestampNs << ','
            << double(pose.timestampNs) * kNsToSeconds << ','
            << pose.tx << ','
            << pose.ty << ','
            << pose.tz << ','
            << pose.qx << ','
            << pose.qy << ','
            << pose.qz << ','
            << pose.qw << ','
            << point.quality << ','
            << pose.poseFrame << '\n';
    }
    if (out.status() != QTextStream::Ok) {
        assignError(error, QStringLiteral("写入轨迹 CSV 失败。"));
        return false;
    }
    assignError(error, QString());
    return true;
}

bool saveTum(const QString& filePath,
             const QVector<SlamTrajectoryPoint>& trajectory,
             QString* error)
{
    if (trajectory.isEmpty()) {
        assignError(error, QStringLiteral("当前没有可导出的 SLAM 轨迹。"));
        return false;
    }

    QFile file(filePath);
    if (!openTextFile(file, error)) {
        return false;
    }

    QTextStream out(&file);
    out.setRealNumberNotation(QTextStream::FixedNotation);
    out.setRealNumberPrecision(9);
    for (const SlamTrajectoryPoint& point : trajectory) {
        const SlamPose& pose = point.pose;
        out << double(pose.timestampNs) * kNsToSeconds << ' '
            << pose.tx << ' '
            << pose.ty << ' '
            << pose.tz << ' '
            << pose.qx << ' '
            << pose.qy << ' '
            << pose.qz << ' '
            << pose.qw << '\n';
    }
    if (out.status() != QTextStream::Ok) {
        assignError(error, QStringLiteral("写入 TUM 轨迹失败。"));
        return false;
    }
    assignError(error, QString());
    return true;
}

} // namespace SlamTrajectoryExport
