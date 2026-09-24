#pragma once

#include <QObject>
#include <QNetworkAccessManager>
#include <QSaveFile>
#include <QVector>
#include <memory>

class QNetworkReply;
class QProgressDialog;
class QWidget;

class UpdateManager : public QObject
{
public:
    explicit UpdateManager(QWidget* window);
    void check(bool manual);

private:
    void fetchRelease(int sourceIndex, bool manual, const QString& lastError = {});
    void download(const QUrl& url, const QString& fileName, qint64 size,
                  const QByteArray& digest);
    void tryDownload(const QVector<QUrl>& sources, int sourceIndex,
                     const QString& path, qint64 size, const QByteArray& digest,
                     QProgressDialog* dialog, const QString& lastError = {});
    void install(const QString& path, QProgressDialog* dialog);
    void showFailure(const QString& message);

    QWidget* window_;
    QNetworkAccessManager network_;
    QNetworkReply* reply_ = nullptr;
    std::unique_ptr<QSaveFile> file_;
};
