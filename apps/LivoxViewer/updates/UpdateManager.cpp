#include "UpdateManager.h"

#include <QApplication>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QMessageBox>
#include <QNetworkReply>
#include <QNetworkRequest>
#include <QProcess>
#include <QProgressDialog>
#include <QRegularExpression>
#include <QStandardPaths>
#include <QStringList>
#include <QUrl>
#include <QVersionNumber>

namespace {

const QUrl kReleaseUrl(QStringLiteral(
    "https://api.github.com/repos/FelixCooper1026/LivoxViewerQT/releases/latest"));

const QStringList kProxyPrefixes = {
    QStringLiteral("https://ghproxy.net/"),
    QStringLiteral("https://gh-proxy.com/"),
    QStringLiteral("https://ghfast.top/"),
    QStringLiteral("https://githubproxy.cc/"),
    QStringLiteral("https://ghproxy.homeboyc.cn/")
};

QVector<QUrl> sourcesFor(const QUrl& origin, bool directFirst)
{
    QVector<QUrl> sources;
    if (directFirst) {
        sources.append(origin);
    }
    for (const QString& prefix : kProxyPrefixes) {
        sources.append(QUrl(prefix + origin.toString(QUrl::FullyEncoded)));
    }
    if (!directFirst) {
        sources.append(origin);
    }
    return sources;
}

QNetworkRequest releaseRequest(const QUrl& url, int timeoutMs)
{
    QNetworkRequest request(url);
    request.setRawHeader("User-Agent", "LivoxViewerQT-Updater");
    request.setAttribute(QNetworkRequest::RedirectPolicyAttribute,
                         QNetworkRequest::NoLessSafeRedirectPolicy);
    request.setTransferTimeout(timeoutMs);
    return request;
}

QString installerName(const QString& version)
{
#ifdef Q_OS_WIN
    return QStringLiteral("LivoxViewerQT_Setup_v%1_x64.exe").arg(version);
#elif defined(Q_OS_LINUX)
    if (qEnvironmentVariableIsSet("APPIMAGE")) {
        return QStringLiteral("LivoxViewerQT-%1-x86_64.AppImage").arg(version);
    }
    return QStringLiteral("livoxviewerqt_%1_amd64.deb").arg(version);
#else
    return {};
#endif
}

bool supportedArchitecture()
{
#if defined(Q_PROCESSOR_X86_64)
    return true;
#else
    return false;
#endif
}

} // namespace

UpdateManager::UpdateManager(QWidget* window)
    : QObject(window)
    , window_(window)
{
}

void UpdateManager::check(bool manual)
{
    if (reply_) {
        return;
    }
    fetchRelease(0, manual);
}

void UpdateManager::fetchRelease(int sourceIndex, bool manual, const QString& lastError)
{
    const QVector<QUrl> sources = sourcesFor(kReleaseUrl, true);
    if (sourceIndex == sources.size()) {
        showFailure(QStringLiteral("无法连接更新服务器，请检查网络连接。\n所有更新线路均不可用：%1")
                        .arg(lastError));
        return;
    }

    QNetworkReply* reply = network_.get(releaseRequest(sources[sourceIndex], 10000));
    reply_ = reply;
    connect(reply, &QNetworkReply::finished, this, [this, reply, sourceIndex, manual]() {
        reply_ = nullptr;
        const QNetworkReply::NetworkError error = reply->error();
        const QString errorText = reply->errorString();
        const QByteArray body = reply->readAll();
        reply->deleteLater();
        if (error != QNetworkReply::NoError) {
            fetchRelease(sourceIndex + 1, manual, errorText);
            return;
        }

        const QJsonObject release = QJsonDocument::fromJson(body).object();
        const QString tag = release.value(QStringLiteral("tag_name")).toString();
        const QRegularExpression versionPattern(QStringLiteral("^v?(\\d+\\.\\d+\\.\\d+)$"));
        const QRegularExpressionMatch match = versionPattern.match(tag);
        if (!match.hasMatch()) {
            fetchRelease(sourceIndex + 1, manual, QStringLiteral("版本信息无效"));
            return;
        }

        const QString version = match.captured(1);
        if (QVersionNumber::fromString(version) <=
            QVersionNumber::fromString(QCoreApplication::applicationVersion())) {
            if (manual) {
                QMessageBox::information(window_, QStringLiteral("检查更新"),
                                         QStringLiteral("当前已是最新版本（%1）")
                                             .arg(QCoreApplication::applicationVersion()));
            }
            return;
        }

        if (!supportedArchitecture() || installerName(version).isEmpty()) {
            showFailure(QStringLiteral("当前平台暂无可用的更新安装包"));
            return;
        }
        const QString name = installerName(version);
        QUrl assetUrl;
        qint64 assetSize = 0;
        QByteArray digest;
        for (const QJsonValue& value : release.value(QStringLiteral("assets")).toArray()) {
            const QJsonObject asset = value.toObject();
            if (asset.value(QStringLiteral("name")).toString() == name) {
                assetUrl = QUrl(asset.value(QStringLiteral("browser_download_url")).toString());
                assetSize = static_cast<qint64>(asset.value(QStringLiteral("size")).toDouble());
                const QRegularExpression digestPattern(QStringLiteral("^sha256:([0-9a-fA-F]{64})$"));
                const QRegularExpressionMatch digestMatch = digestPattern.match(
                    asset.value(QStringLiteral("digest")).toString());
                if (digestMatch.hasMatch()) {
                    digest = digestMatch.captured(1).toLatin1().toLower();
                }
                break;
            }
        }
        if (assetUrl.scheme() != QStringLiteral("https") ||
            assetUrl.host() != QStringLiteral("github.com") || assetSize <= 0) {
            showFailure(QStringLiteral("发现新版本 %1，但发布页缺少适用于本机的安装包").arg(version));
            return;
        }

#ifdef Q_OS_LINUX
        const bool appImage = qEnvironmentVariableIsSet("APPIMAGE");
#else
        const bool appImage = false;
#endif
        const QMessageBox::StandardButton choice = QMessageBox::question(
            window_, QStringLiteral("检测到新版本"),
            (appImage
                ? QStringLiteral("检测到新版本 %1（当前版本 %2），是否下载 AppImage？下载后需手动替换当前文件")
                : QStringLiteral("检测到新版本 %1（当前版本 %2），是否下载并更新？"))
                    .arg(version, QCoreApplication::applicationVersion()),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if (choice == QMessageBox::Yes) {
            download(assetUrl, name, assetSize, digest);
        }
    });
}

void UpdateManager::download(const QUrl& url, const QString& fileName, qint64 size,
                             const QByteArray& digest)
{
    const QString directory = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation)
        + QStringLiteral("/updates");
    if (!QDir().mkpath(directory)) {
        showFailure(QStringLiteral("无法创建更新文件目录：%1").arg(directory));
        return;
    }

    const QString path = directory + QLatin1Char('/') + fileName;
    auto* dialog = new QProgressDialog(QStringLiteral("正在下载更新：%1").arg(fileName),
                                       QStringLiteral("取消"), 0, 100, window_);
    dialog->setWindowTitle(QStringLiteral("更新 LivoxViewerQT"));
    dialog->setWindowModality(Qt::WindowModal);
    dialog->setMinimumDuration(0);
    dialog->setAutoClose(false);
    dialog->setAutoReset(false);
    dialog->show();

    tryDownload(sourcesFor(url, false), 0, path, size, digest, dialog);
}

void UpdateManager::tryDownload(const QVector<QUrl>& sources, int sourceIndex,
                                const QString& path, qint64 size, const QByteArray& digest,
                                QProgressDialog* dialog, const QString& lastError)
{
    if (sourceIndex == sources.size()) {
        dialog->close();
        dialog->deleteLater();
        showFailure(QStringLiteral("所有下载线路均失败：%1").arg(lastError));
        return;
    }

    file_ = std::make_unique<QSaveFile>(path);
    if (!file_->open(QIODevice::WriteOnly)) {
        const QString error = file_->errorString();
        file_.reset();
        dialog->close();
        dialog->deleteLater();
        showFailure(QStringLiteral("无法保存安装包：%1").arg(error));
        return;
    }

    dialog->setLabelText(QStringLiteral("正在下载更新（线路 %1/%2）：%3")
                             .arg(sourceIndex + 1).arg(sources.size()).arg(sources[sourceIndex].host()));
    dialog->setValue(0);
    std::shared_ptr<QCryptographicHash> hash;
    if (!digest.isEmpty()) {
        hash = std::make_shared<QCryptographicHash>(QCryptographicHash::Sha256);
    }
    QNetworkReply* reply = network_.get(releaseRequest(sources[sourceIndex], 20000));
    reply_ = reply;
    connect(dialog, &QProgressDialog::canceled, reply, [reply]() { reply->abort(); });
    connect(reply, &QNetworkReply::readyRead, this, [this, reply, hash]() {
        const QByteArray chunk = reply->readAll();
        if (file_->write(chunk) != chunk.size()) {
            reply->abort();
            return;
        }
        if (hash) {
            hash->addData(chunk);
        }
    });
    connect(reply, &QNetworkReply::downloadProgress, this, [dialog, size](qint64 received, qint64) {
        dialog->setValue(static_cast<int>(qMin<qint64>(100, received * 100 / size)));
    });
    connect(reply, &QNetworkReply::finished, this,
            [this, reply, sources, sourceIndex, path, size, digest, dialog, hash]() {
        reply_ = nullptr;
        const QNetworkReply::NetworkError error = reply->error();
        const QString errorText = reply->errorString();
        const QByteArray remaining = reply->readAll();
        if (!remaining.isEmpty() && file_->write(remaining) == remaining.size() && hash) {
            hash->addData(remaining);
        }
        reply->deleteLater();
        if (dialog->wasCanceled()) {
            file_.reset();
            dialog->close();
            dialog->deleteLater();
            return;
        }
        if (error != QNetworkReply::NoError || file_->error() != QFileDevice::NoError ||
            file_->size() != size || (hash && hash->result().toHex() != digest)) {
            const QString reason = file_->error() != QFileDevice::NoError
                ? file_->errorString()
                : (error != QNetworkReply::NoError ? errorText
                   : (file_->size() != size ? QStringLiteral("文件大小不符")
                                            : QStringLiteral("SHA-256 校验失败")));
            file_.reset();
            tryDownload(sources, sourceIndex + 1, path, size, digest, dialog, reason);
            return;
        }
        if (!file_->commit()) {
            const QString fileError = file_->errorString();
            file_.reset();
            dialog->close();
            dialog->deleteLater();
            showFailure(QStringLiteral("保存安装包失败：%1").arg(fileError));
            return;
        }
        file_.reset();
        install(path, dialog);
    });
}

void UpdateManager::install(const QString& path, QProgressDialog* dialog)
{
#ifdef Q_OS_WIN
    dialog->close();
    dialog->deleteLater();
    if (!QProcess::startDetached(path, {})) {
        showFailure(QStringLiteral("无法启动安装程序。安装包保存在：%1").arg(path));
        return;
    }
    QApplication::quit();
#elif defined(Q_OS_LINUX)
    if (qEnvironmentVariableIsSet("APPIMAGE")) {
        QFile::setPermissions(path, QFile::permissions(path) | QFile::ExeOwner);
        dialog->close();
        dialog->deleteLater();
        QMessageBox::information(window_, QStringLiteral("下载完成"),
                                 QStringLiteral("新版 AppImage 已下载到：\n%1\n请自行替换当前文件").arg(path));
        return;
    }
    dialog->setLabelText(QStringLiteral("正在安装更新，请完成系统授权…"));
    dialog->setCancelButton(nullptr);
    dialog->setRange(0, 0);
    auto* installer = new QProcess(this);
    installer->setProgram(QStringLiteral("pkexec"));
    installer->setArguments({QStringLiteral("/usr/bin/apt-get"),
                             QStringLiteral("install"), QStringLiteral("-y"), path});
    installer->setProcessChannelMode(QProcess::MergedChannels);
    connect(installer, &QProcess::finished, this,
            [this, installer, dialog, path](int exitCode, QProcess::ExitStatus status) {
        const QString output = QString::fromLocal8Bit(installer->readAll()).trimmed();
        installer->deleteLater();
        dialog->close();
        dialog->deleteLater();
        if (status != QProcess::NormalExit || exitCode != 0) {
            showFailure(QStringLiteral("安装更新失败：%1\n安装包保存在：%2")
                            .arg(output.isEmpty() ? QStringLiteral("安装程序退出") : output.right(600), path));
            return;
        }
        QMessageBox::information(window_, QStringLiteral("更新完成"),
                                 QStringLiteral("更新已安装，请重新启动 LivoxViewerQT。"));
        QApplication::quit();
    });
    connect(installer, &QProcess::errorOccurred, this,
            [this, installer, dialog, path](QProcess::ProcessError error) {
        if (error != QProcess::FailedToStart) {
            return;
        }
        const QString reason = installer->errorString();
        installer->deleteLater();
        dialog->close();
        dialog->deleteLater();
        showFailure(QStringLiteral("无法启动安装程序：%1\n安装包保存在：%2").arg(reason, path));
    });
    installer->start();
#else
    Q_UNUSED(path);
    Q_UNUSED(dialog);
#endif
}

void UpdateManager::showFailure(const QString& message)
{
    QMessageBox::warning(window_, QStringLiteral("更新失败"), message);
}
