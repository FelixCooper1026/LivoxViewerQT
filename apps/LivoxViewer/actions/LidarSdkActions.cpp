#include "LivoxViewerWindow.h"

#include "AppConfig/LidarConfigService.h"
#include "AppConfig/NetworkInterfaceService.h"
#include "LivoxCore/LidarDiscoveryService.h"
#include "LivoxCore/LidarSdkService.h"

#include <QApplication>
#include <QFile>
#include <QHostAddress>
#include <QProcess>
#include <QTimer>
#include <QUdpSocket>

#ifndef _WIN32
#include <cerrno>
#include <sys/socket.h>
#include <net/if.h>
#endif

namespace {

QString stateName(LivoxViewerWindow::RealtimeConnectionState state)
{
    switch (state) {
    case LivoxViewerWindow::RealtimeConnectionState::Idle: return "Idle";
    case LivoxViewerWindow::RealtimeConnectionState::WaitingNetwork: return "WaitingNetwork";
    case LivoxViewerWindow::RealtimeConnectionState::Discovering: return "Discovering";
    case LivoxViewerWindow::RealtimeConnectionState::ReconfiguringNetwork: return "ReconfiguringNetwork";
    case LivoxViewerWindow::RealtimeConnectionState::WaitingSdkReady: return "WaitingSdkReady";
    case LivoxViewerWindow::RealtimeConnectionState::InitializingSdk: return "InitializingSdk";
    case LivoxViewerWindow::RealtimeConnectionState::Running: return "Running";
    case LivoxViewerWindow::RealtimeConnectionState::Stopping: return "Stopping";
    case LivoxViewerWindow::RealtimeConnectionState::Error: return "Error";
    }
    return "Unknown";
}

QString netmaskOrDefault(const QString& netmask)
{
    return netmask.isEmpty() ? QString("255.255.255.0") : netmask;
}

QString prefixLengthFromNetmask(const QString& netmask)
{
    const quint32 mask = QHostAddress(netmaskOrDefault(netmask)).toIPv4Address();
    int prefix = 0;
    for (int bit = 31; bit >= 0; --bit) {
        if ((mask >> bit) & 1U) {
            ++prefix;
        }
    }
    return QString::number(prefix);
}

constexpr int kSdkReadyDelayMs = 3000;
constexpr int kSdkInitRetryDelayMs = 1000;
constexpr int kMaxSdkInitRetries = 3;

} // namespace

void LivoxViewerWindow::setRealtimeState(RealtimeConnectionState state)
{
    if (realtimeState == state) {
        return;
    }
    logMessage(QString("[Realtime] State: %1 -> %2").arg(stateName(realtimeState), stateName(state)));
    realtimeState = state;
}

void LivoxViewerWindow::stopAndDeleteTimer(QTimer*& timer)
{
    if (!timer) {
        return;
    }
    timer->stop();
    timer->deleteLater();
    timer = nullptr;
}

bool LivoxViewerWindow::tryGetCurrentDevice(LidarDeviceInfo& out)
{
    QMutexLocker locker(&lidarDeviceMutex);
    if (!hasCurrentLidarHandle) {
        return false;
    }
    auto it = lidarDevices.constFind(currentLidarHandle);
    if (it == lidarDevices.constEnd()) {
        return false;
    }
    out = it.value();
    return true;
}

void LivoxViewerWindow::setCurrentDeviceHandle(uint32_t handle)
{
    currentLidarHandle = handle;
    hasCurrentLidarHandle = true;
}

void LivoxViewerWindow::clearCurrentDevice()
{
    currentLidarHandle = 0;
    hasCurrentLidarHandle = false;
}

std::optional<NetworkInterfaceService::NetworkInterfaceInfo> LivoxViewerWindow::selectedLidarInterface() const
{
    if (!selectedInterfaceName.isEmpty()) {
        const auto selected = NetworkInterfaceService::findInterfaceByName(selectedInterfaceName);
        if (selected.has_value()) {
            return selected;
        }
    }
    if (!selectedNetworkInterfaceSysName.isEmpty()) {
        const auto selected = NetworkInterfaceService::findInterfaceByName(selectedNetworkInterfaceSysName);
        if (selected.has_value()) {
            return selected;
        }
    }
    return std::nullopt;
}

void LivoxViewerWindow::selectLidarInterface(const NetworkInterfaceService::NetworkInterfaceInfo& iface)
{
    selectedInterfaceName = iface.systemName;
    selectedInterfaceDisplayName = iface.displayName;
    selectedHostIp = iface.ipv4;
    selectedNetmask = iface.netmask;
    selectedBroadcast = iface.broadcast;

    selectedNetworkInterfaceSysName = iface.systemName;
    selectedNetworkInterfaceHumanName = iface.displayName;
    selectedNetworkIP = iface.ipv4;
}

std::optional<NetworkInterfaceService::NetworkInterfaceInfo> LivoxViewerWindow::ensureSelectedLidarInterface()
{
    const auto selected = selectedLidarInterface();
    if (selected.has_value()) {
        selectLidarInterface(*selected);
        return selected;
    }

    if (!selectedInterfaceName.isEmpty() || !selectedNetworkInterfaceSysName.isEmpty()) {
        return std::nullopt;
    }

    const QList<NetworkInterfaceService::NetworkInterfaceInfo> interfaces =
        NetworkInterfaceService::availableLidarInterfaces();
    if (interfaces.isEmpty()) {
        selectedInterfaceName.clear();
        selectedInterfaceDisplayName.clear();
        selectedHostIp.clear();
        selectedNetmask.clear();
        selectedBroadcast.clear();
        selectedNetworkInterfaceSysName.clear();
        selectedNetworkInterfaceHumanName.clear();
        selectedNetworkIP.clear();
        return std::nullopt;
    }

    selectLidarInterface(interfaces.first());
    return interfaces.first();
}

void LivoxViewerWindow::enterWaitingNetworkState()
{
    setRealtimeState(RealtimeConnectionState::WaitingNetwork);
    logMessage("[Realtime] No valid lidar network interface. Waiting for network.");

    if (!networkWaitTimer) {
        networkWaitTimer = new QTimer(this);
        connect(networkWaitTimer, &QTimer::timeout, this, [this]() {
            refreshNetworkInterfaces();
            if (!autoDiscoveryEnabled || sdk_initialized || sdk_started) {
                return;
            }
            if (ensureSelectedLidarInterface().has_value()) {
                stopAndDeleteTimer(networkWaitTimer);
                startLidarDiscovery();
            }
        });
    }
    networkWaitTimer->start(2000);
}

void LivoxViewerWindow::scheduleDiscoveryRetry(int delayMs)
{
    if (!discoveryRetryTimer) {
        discoveryRetryTimer = new QTimer(this);
        discoveryRetryTimer->setSingleShot(true);
        connect(discoveryRetryTimer, &QTimer::timeout, this, [this]() {
            if (autoDiscoveryEnabled && !sdk_initialized && !sdk_started) {
                startLidarDiscovery();
            }
        });
    }
    discoveryRetryTimer->start(delayMs);
}

void LivoxViewerWindow::resetDiscoverySessionState()
{
    localIPv4SetForCurrentSession = NetworkInterfaceService::localIpv4Addresses();
    discoverySendCount = 0;
    discoveryBindLogCount = 0;
    sdkInitRetryCount = 0;
    lastAttemptedAutoConfigIp.clear();
    lastDiscoveredLidarType = 0;
    hasLastDiscoveredLidarType = false;
    lastDiscoveredLidarSn.clear();
    lastDiscoveredLidarIp.clear();
}

bool LivoxViewerWindow::createAndBindDiscoverySocket(const NetworkInterfaceService::NetworkInterfaceInfo& iface)
{
    lidarDiscoverySocket = new QUdpSocket(this);

#ifdef _WIN32
    const bool bindOk = lidarDiscoverySocket->bind(QHostAddress(iface.ipv4), 56000,
                                                   QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
#else
    const bool bindOk = lidarDiscoverySocket->bind(QHostAddress::AnyIPv4, 56000,
                                                   QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
#endif

    if (!bindOk) {
        logMessage(QString("[Discovery] Bind UDP socket on %1/%2:56000 failed: %3")
                       .arg(iface.displayName, iface.ipv4, lidarDiscoverySocket->errorString()));
        lidarDiscoverySocket->deleteLater();
        lidarDiscoverySocket = nullptr;
        return false;
    }

    logMessage(QString("[Discovery] Bind UDP socket on %1/%2:56000 success")
                   .arg(iface.displayName, iface.ipv4));

    QUdpSocket* sock = lidarDiscoverySocket;
    connect(sock, &QUdpSocket::readyRead, this, [this, sock]() {
        while (sock->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(sock->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort = 0;
            sock->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            const QString senderIp = sender.toString();
            const QString localIp = sock->localAddress().toString();
            if (LidarDiscoveryService::isLocalDatagram(senderIp, localIp, localIPv4SetForCurrentSession)) {
                continue;
            }

            onLidarDiscoveryResponse(datagram, sender);
        }
    });

    return true;
}

void LivoxViewerWindow::startLidarDiscovery()
{
    if (realtimeState == RealtimeConnectionState::Discovering || lidarDiscoveryActive) {
        return;
    }
    if (sdk_started || sdk_initialized) {
        logMessage("[Discovery] SDK is already running; discovery requires SDK restart.");
        return;
    }

    auto iface = ensureSelectedLidarInterface();
    if (!iface.has_value()) {
        enterWaitingNetworkState();
        return;
    }

    stopAndDeleteTimer(networkWaitTimer);
    stopAndDeleteTimer(discoveryRetryTimer);
    stopLidarDiscovery();
    resetDiscoverySessionState();

    if (!createAndBindDiscoverySocket(*iface)) {
        logMessage("[Discovery] Bind failed. Will retry in 5 seconds.");
        setRealtimeState(RealtimeConnectionState::Idle);
        scheduleDiscoveryRetry(5000);
        return;
    }

    discoveryBroadcastTimer = new QTimer(this);
    connect(discoveryBroadcastTimer, &QTimer::timeout, this, &LivoxViewerWindow::sendLidarBroadcastDiscovery);

    discoveryTimeoutTimer = new QTimer(this);
    discoveryTimeoutTimer->setSingleShot(true);
    connect(discoveryTimeoutTimer, &QTimer::timeout, this, [this]() {
        logMessage("[Discovery] Timeout. Will retry in 5 seconds.");
        stopLidarDiscovery();
        scheduleDiscoveryRetry(5000);
    });

    lidarDiscoveryActive = true;
    setRealtimeState(RealtimeConnectionState::Discovering);
    sendLidarBroadcastDiscovery();
    discoveryBroadcastTimer->start(3000);
    discoveryTimeoutTimer->start(30000);
}

void LivoxViewerWindow::stopLidarDiscovery()
{
    lidarDiscoveryActive = false;
    stopAndDeleteTimer(discoveryBroadcastTimer);
    stopAndDeleteTimer(discoveryTimeoutTimer);
    stopAndDeleteTimer(lidarDiscoveryTimer);

    if (lidarDiscoverySocket) {
        lidarDiscoverySocket->disconnect(this);
        lidarDiscoverySocket->close();
        lidarDiscoverySocket->deleteLater();
        lidarDiscoverySocket = nullptr;
    }

    localIPv4SetForCurrentSession.clear();
    discoverySendCount = 0;
    discoveryBindLogCount = 0;
    sdkInitRetryCount = 0;
    lastAttemptedAutoConfigIp.clear();

    if (realtimeState == RealtimeConnectionState::Discovering) {
        setRealtimeState(RealtimeConnectionState::Idle);
    }
}

void LivoxViewerWindow::sendLidarBroadcastDiscovery()
{
    if (!lidarDiscoverySocket || !lidarDiscoveryActive) {
        return;
    }

    const QByteArray discoveryCmd = LidarDiscoveryService::discoveryCommand();
    QHostAddress target = QHostAddress::Broadcast;
    const auto iface = selectedLidarInterface();

#ifndef _WIN32
    if (iface.has_value()) {
        const qintptr sockfd = lidarDiscoverySocket->socketDescriptor();
        if (sockfd != -1) {
            const QByteArray ifaceName = iface->systemName.toLocal8Bit();
            if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifaceName.constData(), ifaceName.length()) == 0 &&
                discoveryBindLogCount == 0) {
                logMessage(QString("[Discovery] Socket bound to interface %1").arg(iface->systemName));
                ++discoveryBindLogCount;
            }
        }
    }
#endif

    const qint64 written = lidarDiscoverySocket->writeDatagram(discoveryCmd, target, 56000);
    ++discoverySendCount;
    if (written < 0) {
        logMessage(QString("[Discovery] Broadcast failed: %1").arg(lidarDiscoverySocket->errorString()));
        return;
    }
    if (discoverySendCount == 1 || discoverySendCount % 10 == 0) {
        logMessage(QString("[Discovery] Broadcast sent to %1:56000").arg(target.toString()));
    }
}

void LivoxViewerWindow::onLidarDiscoveryResponse(const QByteArray& data, const QHostAddress& sender)
{
    const LidarDiscoveryService::DiscoveryResponse response =
        LidarDiscoveryService::parseDiscoveryResponse(data);
    if (!response.valid) {
        logMessage(QString("[Discovery] Ignore invalid response from %1: %2")
                       .arg(sender.toString(), response.errorMessage));
        return;
    }
    handleParsedDiscoveryResponse(response, sender);
}

void LivoxViewerWindow::handleParsedDiscoveryResponse(const LidarDiscoveryService::DiscoveryResponse& response,
                                                      const QHostAddress& sender)
{
    lastDiscoveredLidarType = response.deviceType;
    hasLastDiscoveredLidarType = true;
    lastDiscoveredLidarSn = response.serialNumber;
    lastDiscoveredLidarIp = response.deviceIp;

    logMessage(QString("[Discovery] Valid ACK from %1, SN=%2, type=%3, cmdPort=%4")
                   .arg(response.deviceIp, response.serialNumber)
                   .arg(response.deviceType)
                   .arg(response.commandPort));
    Q_UNUSED(sender);

    const auto iface = selectedLidarInterface();
    if (!iface.has_value()) {
        enterWaitingNetworkState();
        return;
    }

    const bool ipConflict = response.deviceIp == iface->ipv4;
    const bool sameSubnet = NetworkInterfaceService::isInSameSubnet(response.deviceIp,
                                                                    iface->ipv4,
                                                                    netmaskOrDefault(iface->netmask));
    if (ipConflict || !sameSubnet) {
        logMessage(QString("[Discovery] Device and host are not in same subnet. host=%1, device=%2")
                       .arg(iface->ipv4, response.deviceIp));

        const QString newHostIp = calculateCompatibleHostIP(response.deviceIp);
        if (newHostIp.isEmpty()) {
            scheduleDiscoveryRetry(5000);
            return;
        }
        if (!autoConfigHostIpEnabled) {
            logMessage(QString("[Network] Auto host IP config disabled. Suggested host IP: %1").arg(newHostIp));
            stopLidarDiscovery();
            scheduleDiscoveryRetry(5000);
            return;
        }
        if (lastAttemptedAutoConfigIp == newHostIp) {
            logMessage(QString("[Network] IP %1 already attempted in this discovery session").arg(newHostIp));
            stopLidarDiscovery();
            scheduleDiscoveryRetry(5000);
            return;
        }

        lastAttemptedAutoConfigIp = newHostIp;
        stopLidarDiscovery();
        updateHostIPForDeviceAsync(*iface, newHostIp, netmaskOrDefault(iface->netmask));
        return;
    }

    logMessage(QString("[Discovery] Device and host are in same subnet. host=%1, device=%2")
                   .arg(iface->ipv4, response.deviceIp));
    stopLidarDiscovery();
    QTimer::singleShot(500, this, &LivoxViewerWindow::initializeLivoxSdk);
}

QString LivoxViewerWindow::calculateCompatibleHostIP(const QString& deviceIP)
{
    const QString candidate = NetworkInterfaceService::calculateCompatibleHostIp(deviceIP);
    if (!candidate.isEmpty()) {
        logMessage(QString("[Network] Compatible host IP candidate: %1").arg(candidate));
    } else {
        logMessage(QString("[Network] No available compatible host IP candidate for device %1").arg(deviceIP));
    }
    return candidate;
}

bool LivoxViewerWindow::updateHostIPForDevice(const QString& deviceIP)
{
    const auto iface = selectedLidarInterface();
    if (!iface.has_value()) {
        return false;
    }
    const QString newHostIp = calculateCompatibleHostIP(deviceIP);
    if (newHostIp.isEmpty()) {
        return false;
    }
    updateHostIPForDeviceAsync(*iface, newHostIp, netmaskOrDefault(iface->netmask));
    return true;
}

void LivoxViewerWindow::updateHostIPForDeviceAsync(const NetworkInterfaceService::NetworkInterfaceInfo& iface,
                                                   const QString& targetHostIp,
                                                   const QString& netmask)
{
    setRealtimeState(RealtimeConnectionState::ReconfiguringNetwork);
    logMessage(QString("[Network] Start async IP reconfiguration: iface=%1, newIp=%2")
                   .arg(iface.displayName, targetHostIp));

    const QString interfaceName = iface.systemName;
    QProcess* process = new QProcess(this);
    connect(process, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, [this, process, interfaceName, targetHostIp](int exitCode, QProcess::ExitStatus exitStatus) {
        const QString stdoutText = QString::fromLocal8Bit(process->readAllStandardOutput()).trimmed();
        const QString stderrText = QString::fromLocal8Bit(process->readAllStandardError()).trimmed();
        process->deleteLater();

        if (exitStatus == QProcess::NormalExit && exitCode == 0) {
            logMessage("[Network] IP reconfiguration command completed");
            if (!updateConfigFileIP(targetHostIp)) {
                setRealtimeState(RealtimeConnectionState::Error);
                return;
            }
            waitForHostIpThenInitializeSdk(interfaceName, targetHostIp, 20);
            return;
        }

        logMessage(QString("[Network] IP reconfiguration failed. exit=%1 stdout=%2 stderr=%3")
                       .arg(exitCode)
                       .arg(stdoutText, stderrText));
        setRealtimeState(RealtimeConnectionState::Error);
        scheduleDiscoveryRetry(5000);
    });

#ifdef _WIN32
    const QString ifaceName = iface.displayName.isEmpty() ? iface.systemName : iface.displayName;
    QStringList arguments;
    arguments << "interface" << "ip" << "set" << "address"
              << ("name=" + ifaceName)
              << "static" << targetHostIp << netmaskOrDefault(netmask);
    process->start("netsh", arguments);
#else
    const QString prefix = prefixLengthFromNetmask(netmask);
    const QString cidr = QString("%1/%2").arg(targetHostIp, prefix);
    const QString script =
        "set -e; "
        "ip -4 addr flush dev \"$1\" scope global; "
        "ip addr add \"$2\" dev \"$1\"; "
        "ip link set dev \"$1\" up";
    process->start("pkexec", QStringList() << "/bin/sh" << "-c" << script
                                           << "livox-ip-config" << iface.systemName << cidr);
#endif
}

void LivoxViewerWindow::waitForHostIpThenInitializeSdk(const QString& interfaceName,
                                                       const QString& targetHostIp,
                                                       int remainingAttempts)
{
    const auto iface = NetworkInterfaceService::findInterfaceByName(interfaceName);
    if (iface.has_value() && iface->ipv4 == targetHostIp) {
        selectLidarInterface(*iface);
        refreshNetworkInterfaces();
        sdkInitRetryCount = 0;
        setRealtimeState(RealtimeConnectionState::WaitingSdkReady);
        logMessage(QString("[Network] IP reconfiguration success: host IP %1 is active on %2. Initialize SDK without rediscovery after delay.")
                       .arg(targetHostIp, iface->displayName));
        QTimer::singleShot(kSdkReadyDelayMs, this, &LivoxViewerWindow::initializeLivoxSdk);
        return;
    }

    if (remainingAttempts <= 0) {
        logMessage(QString("[Network] Host IP %1 is not active yet. Restart discovery.").arg(targetHostIp));
        setRealtimeState(RealtimeConnectionState::Idle);
        scheduleDiscoveryRetry(5000);
        return;
    }

    QTimer::singleShot(500, this, [this, interfaceName, targetHostIp, remainingAttempts]() {
        waitForHostIpThenInitializeSdk(interfaceName, targetHostIp, remainingAttempts - 1);
    });
}

void LivoxViewerWindow::initializeLivoxSdk()
{
    logMessage("[SDK] LivoxLidarSdkInit requested");

    if (sdk_initialized || sdk_started) {
        logMessage("[SDK] Already initialized");
        setRealtimeState(RealtimeConnectionState::Running);
        return;
    }

    setRealtimeState(RealtimeConnectionState::InitializingSdk);

    if (lidarDiscoveryActive || lidarDiscoverySocket) {
        stopLidarDiscovery();
        logMessage("[SDK] Discovery socket stopped before SDK init");
    }
    if (lidarDiscoverySocket) {
        logMessage("[SDK] Failed to stop discovery socket. Abort SDK initialization.");
        setRealtimeState(RealtimeConnectionState::Error);
        return;
    }

    const auto iface = selectedLidarInterface();
    if (!iface.has_value()) {
        logMessage("[SDK] No valid lidar network interface. Abort SDK initialization.");
        enterWaitingNetworkState();
        return;
    }

    QString migrationNote;
    QString configPath = LidarConfigService::resolveConfigJsonPath(&migrationNote);
    if (!migrationNote.isEmpty()) {
        logMessage(migrationNote);
    }

    if (configPath.isEmpty() || !QFile::exists(configPath)) {
        logMessage("[Config] config.json not found. Opening config generator.");
        if (!showConfigGeneratorDialog()) {
            logMessage("[Config] config.json generation canceled. Abort SDK initialization.");
            setRealtimeState(RealtimeConnectionState::Error);
            return;
        }
        configPath = LidarConfigService::resolveConfigJsonPath();
    }

    if (configPath.isEmpty() || !QFile::exists(configPath)) {
        logMessage("[Config] config.json not found. Abort SDK initialization.");
        setRealtimeState(RealtimeConnectionState::Error);
        return;
    }

    QString netCheckDetails;
    if (!LidarConfigService::checkNetworkCompatibility(configPath, &netCheckDetails, iface->ipv4)) {
        if (!netCheckDetails.isEmpty()) {
            logMessage(netCheckDetails);
        }
        if (!updateConfigFileIP(iface->ipv4)) {
            logMessage("[Config] Failed to update host_ip in config.json. Abort SDK initialization.");
            setRealtimeState(RealtimeConnectionState::Error);
            return;
        }
    } else if (!netCheckDetails.isEmpty()) {
        logMessage(netCheckDetails);
    }

    if (!updateConfigFileDeviceTypeIfNeeded(configPath)) {
        logMessage("[Config] Device type update failed; continue with current config.");
    }

    logMessage(QString("[SDK] LivoxLidarSdkInit start: %1").arg(configPath));
    if (!LidarSdkService::initialize(configPath)) {
        LidarSdkService::clearCallbacks();
        LidarSdkService::shutdown();
        sdk_started = false;
        sdk_initialized = false;
        pointCloudCallbackEnabled = false;

        if (sdkInitRetryCount < kMaxSdkInitRetries) {
            ++sdkInitRetryCount;
            logMessage(QString("[SDK] LivoxLidarSdkInit failed. Retry SDK init in %1 ms (%2/%3).")
                           .arg(kSdkInitRetryDelayMs)
                           .arg(sdkInitRetryCount)
                           .arg(kMaxSdkInitRetries));
            setRealtimeState(RealtimeConnectionState::WaitingSdkReady);
            QTimer::singleShot(kSdkInitRetryDelayMs, this, &LivoxViewerWindow::initializeLivoxSdk);
            return;
        }

        logMessage("[SDK] LivoxLidarSdkInit failed repeatedly. Fallback to discovery.");
        sdkInitRetryCount = 0;
        setRealtimeState(RealtimeConnectionState::Idle);
        scheduleDiscoveryRetry(5000);
        return;
    }

    sdkInitRetryCount = 0;
    sdk_initialized = true;
    logMessage("[SDK] LivoxLidarSdkInit success");
    logMessage(QString("Livox SDK version: %1").arg(LidarSdkService::versionString()));

    LidarSdkService::CallbackSet callbacks;
    callbacks.infoChange = onLidarDeviceInfoChange;
    callbacks.pointCloud = onPointCloudData;
    callbacks.imu = onImuData;
    callbacks.statusInfo = onStatusInfo;
    callbacks.clientData = this;
    LidarSdkService::registerCallbacks(callbacks);

    sdk_started = true;
    pointCloudCallbackEnabled = true;
    if (statusLabelBar) {
        statusLabelBar->setText("已连接 - 采样中");
    }
    setRealtimeState(RealtimeConnectionState::Running);
}

void LivoxViewerWindow::shutdownLivoxSdk()
{
    if (!sdk_started && !sdk_initialized) {
        return;
    }

    setRealtimeState(RealtimeConnectionState::Stopping);
    shutting_down = true;
    pointCloudCallbackEnabled = false;
    LidarSdkService::clearCallbacks();

    {
        QMutexLocker locker(&lidarDeviceMutex);
        lidarDevices.clear();
    }
    {
        QMutexLocker locker(&frameMutex);
        pendingFrames.clear();
        lastSeenTimestamp.clear();
        lastFrameTimestamp.clear();
    }
    clearCurrentDevice();
    updateLidarDeviceList();

    LidarSdkService::shutdown();

    sdk_started = false;
    sdk_initialized = false;
    shutting_down = false;
    setRealtimeState(RealtimeConnectionState::Idle);
    logMessage("[SDK] Livox SDK shutdown complete");
}

void LivoxViewerWindow::restartRealtimeConnectionForNetworkChange()
{
    logMessage("[Realtime] Network changed while SDK running. Restart SDK.");
    pointCloudCallbackEnabled = false;
    stopLidarDiscovery();
    shutdownLivoxSdk();

    const auto iface = selectedLidarInterface();
    if (iface.has_value()) {
        updateConfigFileIP(iface->ipv4);
    }

    setRealtimeState(RealtimeConnectionState::Idle);
    startLidarDiscovery();
}

bool LivoxViewerWindow::updateConfigFileIP(const QString& newHostIP)
{
    QString message;
    const bool updated = LidarConfigService::updateResolvedConfigHostIp(newHostIP, &message);
    if (!message.isEmpty()) {
        logMessage(message);
    }
    return updated;
}

bool LivoxViewerWindow::updateConfigFileDeviceTypeIfNeeded(const QString& configPath)
{
    QString message;
    const bool updated = LidarConfigService::updateDeviceTypeIfNeeded(configPath,
                                                                      hasLastDiscoveredLidarType,
                                                                      lastDiscoveredLidarType,
                                                                      &message);
    if (!message.isEmpty()) {
        logMessage(message);
    }
    return updated;
}
