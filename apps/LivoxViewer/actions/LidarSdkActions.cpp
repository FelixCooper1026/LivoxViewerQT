#include "LivoxViewerWindow.h"
#include "AppConfig/LidarConfigService.h"
#include "AppConfig/NetworkInterfaceService.h"
#include "LivoxCore/LidarDiscoveryService.h"
#include "LivoxCore/LidarSdkService.h"
#include <QDir>
#include <QApplication>
#include <QFile>
#include <QStringList>
#include <QNetworkInterface>
#include <QHostAddress>
#include <QAbstractSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonParseError>
#include <QDebug>
#include <QJsonArray>
#include <QUdpSocket>
#include <QTimer>
#include <QProcess>
#include <QStandardPaths>

#ifndef _WIN32
#include <sys/socket.h>
#include <net/if.h>
#include <cerrno>
#endif

// 添加网段检查的辅助函数
bool isInSameSubnet(const QString &hostIP, const QString &currentIP, const QString &subnetMask)
{
    return NetworkInterfaceService::isInSameSubnet(hostIP, currentIP, subnetMask);
}
QString getCurrentHostIP(const QString& selectedIP = QString())
{
    return NetworkInterfaceService::currentHostIp(selectedIP);
}
static bool hasWiredNetworkDeviceConnected()
{
    return NetworkInterfaceService::hasWiredNetworkDeviceConnected();
}
static QString resolveConfigJsonPath(QString* migrationNote = nullptr)
{
    return LidarConfigService::resolveConfigJsonPath(migrationNote);
}
bool checkConfigFileNetworkCompatibility(const QString &configPath, QString *details = nullptr, const QString &selectedIP = QString())
{
    return LidarConfigService::checkNetworkCompatibility(configPath, details, selectedIP);
}
void LivoxViewerWindow::initializeLivoxSdk()
{
    logMessage("开始初始化Livox SDK...");

    if (sdk_initialized || sdk_started)
    {
        logMessage("Livox SDK 已初始化，跳过");
        return;
    }

    // 1) 优先检查有线网口是否有设备连接
    if (!hasWiredNetworkDeviceConnected())
    {
        logMessage("网口未连接任何设备，跳过 SDK 初始化与配置文件生成");
        return;
    }

    // 2) 如果设备发现仍在进行，等待其完成
    if (lidarDiscoveryActive)
    {
        logMessage("设备发现仍在进行中，等待完成...");
        // 延迟重试，但不要无限等�
        static int retryCount = 0;
        if (retryCount < 10)
        { // 最多等待20秒
            retryCount++;
            QTimer::singleShot(2000, this, &LivoxViewerWindow::initializeLivoxSdk);
            return;
        }
        else
        {
            logMessage("设备发现等待超时，强制继续SDK初始化");
            retryCount = 0;
        }
    }

    // 3) 检查网络状态
    QString currentIP = getSelectedHostIP();
    if (currentIP.isEmpty())
    {
        logMessage("错误: 无法获取当前主机IP，SDK初始化失败");
        return;
    }
    logMessage(QString("当前主机IP: %1，继续SDK初始化...").arg(currentIP));

    // 3) 查找配置文件（修复逻辑：增加 exists 校验）
    QString migrationNote;
    QString configPath = resolveConfigJsonPath(&migrationNote);
    if (!migrationNote.isEmpty()) {
        logMessage(migrationNote);
    }

    // 即使路径不为空，如果文件不存在，也要进入向导
    if (configPath.isEmpty() || !QFile::exists(configPath))
    {
        logMessage("未找到配置文件 config.json，将打开配置向导");
        if (!showConfigGeneratorDialog())
        {
            logMessage("已取消生成配置文件，SDK 初始化终止");
            return;
        }

        // 生成后再次尝试解析（或直接使用之前的 configPath，因为 resolve 已经创建了目录）
        if (!QFile::exists(configPath))
        {
            logMessage("错误: 配置向导运行后仍未检测到生成的 config.json");
            return;
        }
    }

    logMessage("找到配置文件: " + configPath);

    // 4) 自动更新配置：检查配置文件 IP 匹配，如不一致则自动修正为当前主机IP
    logMessage("开始检查配置文件 IP 匹配...");
    {
        QString netCheckDetails;
        bool ok = checkConfigFileNetworkCompatibility(configPath, &netCheckDetails, getSelectedHostIP());
        if (!ok)
        {
            if (!netCheckDetails.isEmpty())
                logMessage(netCheckDetails);
            else
                logMessage("配置文件中的host_ip与当前主机IP不一致");

            const QString currentHostIP = getSelectedHostIP();
            if (currentHostIP.isEmpty())
            {
                logMessage("错误: 无法获取当前主机IP，无法自动修正配置文件");
                return;
            }

            logMessage(QString("检测到配置文件 host_ip 与当前主机 IP 不一致，将自动更新所有 host_ip 为: %1").arg(currentHostIP));
            if (!updateConfigFileIP(currentHostIP))
            {
                logMessage("自动更新配置文件 host_ip 失败，如需手动调整可通过菜单“生成配置文件...”重新生成");
            }
            else
            {
                logMessage("已自动更新配置文件中的 host_ip 字段，将使用更新后的配置继续初始化 SDK");
            }
        }
        else
        {
            if (!netCheckDetails.isEmpty())
                logMessage(netCheckDetails);
        }
    }

    // 5) 根据已发现的雷达型号自动校正配置文件中的 Device type（顶层设备键）并初始化 SDK
    if (!updateConfigFileDeviceTypeIfNeeded(configPath))
    {
        logMessage("自动校正配置文件 Device type 失败，继续使用原配置尝试初始化 SDK");
    }

    try
    {
        if (!LidarSdkService::initialize(configPath))
        {
            logMessage("错误: Livox SDK 初始化失败");
            return;
        }

        sdk_initialized = true;
        logMessage("Livox SDK 初始化成功");
        // 获取并打印 SDK 版本信息
        logMessage(QString("Livox SDK version: %1").arg(LidarSdkService::versionString()));

        // 设置回调函数
        LidarSdkService::CallbackSet callbacks;
        callbacks.infoChange = onLidarDeviceInfoChange;
        callbacks.pointCloud = onPointCloudData;
        callbacks.imu = onImuData;
        callbacks.statusInfo = onStatusInfo;
        callbacks.clientData = this;
        LidarSdkService::registerCallbacks(callbacks);

        sdk_started = true;
        pointCloudCallbackEnabled = true; // 自动开始采样
        statusLabelBar->setText("已连接 - 采样中");

        QTimer::singleShot(5000, this, [this]()
                           {
            if (lidarDevices.isEmpty()) {
            } else {
            } });
    }
    catch (...)
    {
        return;
    }
}

void LivoxViewerWindow::shutdownLivoxSdk()
{
    if (!sdk_started && !sdk_initialized)
    {
        return;
    }

    // 标记关闭中：回调进入时直接返回（在回调开头检查）
    shutting_down = true;

    // 先解除所有回调注册，防止 Uninit 过程中回调进入
    LidarSdkService::clearCallbacks();


    // 清空UI侧设备状态
    {
        QMutexLocker locker(&lidarDeviceMutex);
        lidarDevices.clear();
    }
    if (lidarDeviceList)
        lidarDeviceList->clear();
    currentLidarDevice = nullptr;

    // 最后调用 Uninit
    LidarSdkService::shutdown();

    sdk_started = false;
    sdk_initialized = false;
    shutting_down = false; // 退出完成
    logMessage("Livox SDK 已清理");
}

// 设备发现相关方法实现

void LivoxViewerWindow::startLidarDiscovery()
{
    if (lidarDiscoveryActive)
    {
        return;
        logMessage("设备发现已启动");
    }

    // 检查是否有有线网口连接
    if (!hasWiredNetworkDeviceConnected())
    {
        logMessage("未检测到有线网口连接，等待设备接入...");

        // 在此处创建一个一次性的定时器，轮询等待网口上线
        static QTimer *waitTimer = nullptr;

        if (!waitTimer)
        {
            waitTimer = new QTimer(this);
            waitTimer->setInterval(2000); // 每2秒检测一次
            connect(waitTimer, &QTimer::timeout, this, [this]() {
                if (hasWiredNetworkDeviceConnected())
                {
                    logMessage("检测到有线网口已连接，自动启动设备发现...");
                    waitTimer->stop();
                    startLidarDiscovery(); // 重新进入发现流程
                }
            });
            waitTimer->start();
        }

        return; // 暂时退出，等待下一轮检测
        logMessage ("等待有线网口连接中...");
    }

    // 获取有线网口的 IPv4（必须存在）
    QString hwIp = getSelectedHostIP();

    // ✅ 如果没有有效IP（空或169.254.x.x），则自动为网口分配固定IP
    if (hwIp.isEmpty() || hwIp.startsWith("169.254."))
    {
        logMessage("检测到有线网口已连接，但未分配有效IPv4，尝试自动配置固定IP...");

        QString targetIfaceName;
        for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces())
        {
            if ((iface.flags() & QNetworkInterface::IsUp) &&
                (iface.flags() & QNetworkInterface::IsRunning) &&
                !(iface.flags() & QNetworkInterface::IsLoopBack) &&
                !(iface.flags() & QNetworkInterface::IsPointToPoint))
            {

                QString name = iface.humanReadableName().toLower();
                QString sysName = iface.name().toLower();

                // 排除无线和蓝牙
                if (name.contains("wifi") || name.contains("wlan") ||
                    name.contains("wireless") || name.contains("蓝牙"))
                    continue;
                // 只选择以太网接口
                if (!(sysName.startsWith("ethernet") || name.contains("以太网") || name.contains("ethernet")))
                    continue;

                targetIfaceName = iface.humanReadableName();
                break;
            }
        }

        if (!targetIfaceName.isEmpty())
        {
#ifdef _WIN32
            QString newIPBase = "192.168.2.";
            QString mask = "255.255.255.0";
            int startIndex = 2; // 初始 IP 后缀
            int maxTry = 10;    // 最多尝试 10 个 IP

            bool ipSetSuccess = false;

            for (int i = startIndex; i < startIndex + maxTry; ++i)
            {
                QString newIP = newIPBase + QString::number(i);

                logMessage(QString("正在为接口 \"%1\" 自动设置固定IP: %2/%3")
                               .arg(targetIfaceName)
                               .arg(newIP)
                               .arg(mask));
                logMessage("注意: Windows 下修改网络配置需要管理员权限");

                QProcess process;
                QStringList args;
                args << "interface" << "ip" << "set" << "address"
                     << targetIfaceName
                     << "source=static"
                     << ("addr=" + newIP)
                     << ("mask=" + mask);

                // 打印完整命令（调试用）
                QString fullCmd = QString("netsh %1").arg(args.join(" "));
                logMessage(QString("执行命令: %1").arg(fullCmd));

                process.start("netsh", args);
                process.waitForFinished(8000);

                QByteArray err = process.readAllStandardError();
                QByteArray out = process.readAllStandardOutput();

                logMessage(QString("netsh 输出: %1").arg(QString::fromLocal8Bit(out)));
                logMessage(QString("netsh 错误: %1").arg(QString::fromLocal8Bit(err)));
                logMessage(QString("netsh 退出码: %1").arg(process.exitCode()));

                if (process.exitCode() == 0)
                {
                    logMessage(QString("已成功为接口 \"%1\" 设置固定IP: %2").arg(targetIfaceName).arg(newIP));

                    // 等待网络刷新
                    for (int j = 0; j < 10; ++j)
                    {
                        QThread::msleep(500);
                        QString refreshedIP = getSelectedHostIP();
                        if (refreshedIP == newIP)
                        {
                            hwIp = refreshedIP;
                            break;
                        }
                    }

                    if (hwIp != newIP)
                    {
                        logMessage("警告: IP 已设置但系统尚未刷新，等待中...");
                        QThread::sleep(2);
                        hwIp = getSelectedHostIP();
                    }

                    ipSetSuccess = true;
                    break; // 成功，退出循环
                }
                else
                {
                    logMessage(QString("尝试设置IP %1 失败: %2").arg(newIP).arg(QString::fromLocal8Bit(err)));
                    logMessage("将尝试下一个可用 IP...");
                }
            }

            if (!ipSetSuccess)
            {
                logMessage("所有尝试的 IP 均失败，请以管理员权限运行程序或手动设置网口IP后重试");
                return;
            }
#endif
        }
        else
        {
            logMessage("未找到有效的有线网口接口名，无法自动分配IP");
            return;
        }
    }

    // 再次验证是否拿到有效IP
    if (hwIp.isEmpty())
    {
        logMessage("错误: 无法获取有效IPv4地址，跳过设备发现");
        return;
    }

    logMessage(QString("使用有线接口 IPv4 地址: %1").arg(hwIp));

    // 创建UDP socket
    lidarDiscoverySocket = new QUdpSocket(this);

    // 尝试绑定到 56000 端口，重试5次，每次间隔1秒
    int retryCount = 5;
    bool bindOk = false;
    for (int i = 0; i < retryCount; ++i)
    {
    #ifdef _WIN32
        // Windows 原有逻辑：绑定到具体的有线网口 IP
        bindOk = lidarDiscoverySocket->bind(QHostAddress(hwIp), 56000,
                                    QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    #else
        // Linux 修复逻辑：绑定到 AnyIPv4 以接收设备广播的 ACK
        bindOk = lidarDiscoverySocket->bind(QHostAddress::AnyIPv4, 56000,
                                    QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    #endif

        if (bindOk)
        {
            break;
        }
        else
        {
    #ifdef _WIN32
            logMessage(QString("警告: 绑定到 %1:56000 失败 (第%2次重试): %3")
                        .arg(hwIp).arg(i + 1).arg(lidarDiscoverySocket->errorString()));
    #else
            logMessage(QString("警告: 绑定到 AnyIPv4:56000 失败 (第%1次重试): %2")
                        .arg(i + 1).arg(lidarDiscoverySocket->errorString()));
    #endif
            QThread::sleep(1);
        }
    }

    if (!bindOk)
    {
    #ifdef _WIN32
        logMessage(QString("警告: 绑定到 %1:56000 失败: %2").arg(hwIp).arg(lidarDiscoverySocket->errorString()));
        // 最小回退：尝试绑定到 AnyIPv4
        if (!lidarDiscoverySocket->bind(QHostAddress::AnyIPv4, 56000,
                                QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
        {
            logMessage("警告: 回退绑定到 AnyIPv4 仍失败，无法启动设备发现");
            delete lidarDiscoverySocket;
            lidarDiscoverySocket = nullptr;
            return;
        }
        else
        {
            logMessage("已回退绑定到 AnyIPv4:56000（注意：广播可能不会从有线接口发出）");
        }
    #else
        logMessage("错误: 绑定到 AnyIPv4:56000 失败: " + lidarDiscoverySocket->errorString());
        delete lidarDiscoverySocket;
        lidarDiscoverySocket = nullptr;
        return;
    #endif
    }
    else
    {
    #ifdef _WIN32
        logMessage(QString("已绑定 discovery socket 到 %1:56000（将使用该接口发送广播）").arg(hwIp));
    #else
        logMessage("已绑定 discovery socket 到 AnyIPv4:56000（准备监听雷达响应）");
    #endif
    }

    // 连接readyRead信号
    // 注意：不要在此 lambda 中依赖成员 lidarDiscoverySocket（stopLidarDiscovery 可能将其置空）
    QUdpSocket* sock = lidarDiscoverySocket;
    connect(sock, &QUdpSocket::readyRead, this, [this, sock]() {
        while (sock && sock->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(sock->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort = 0;
            sock->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            QString senderIP = sender.toString();
            QString localIP = sock->localAddress().toString();

            // === Step 1: 构建本机所有IPv4地址集合 ===
            static QSet<QString> localIPv4Set;
            if (localIPv4Set.isEmpty()) {
                localIPv4Set = NetworkInterfaceService::localIpv4Addresses();
            }

            if (LidarDiscoveryService::isLocalDatagram(senderIP, localIP, localIPv4Set)) {
                logMessage(QString("忽略来自本机接口的数据包: %1 (绑定接口: %2)")
                               .arg(senderIP)
                               .arg(localIP));
                continue;
            }

            // === Step 5: 额外防护：当主机与设备IP完全一致时，禁止忽略 ===
            QString currentHostIP = getSelectedHostIP();
            if (senderIP == currentHostIP) {
                logMessage(QString("警告: 收到与主机相同IP(%1)的UDP包，可能是雷达设备冲突，仍尝试解析").arg(senderIP));
            }

            // === Step 6: 正常解析 ===
            onLidarDiscoveryResponse(datagram, sender);
        }
    });


    // 创建定时器，定期发送广播发现命令
    lidarDiscoveryTimer = new QTimer(this);
    connect(lidarDiscoveryTimer, &QTimer::timeout, this, &LivoxViewerWindow::sendLidarBroadcastDiscovery);

    // 创建超时定时器，30秒后自动停止设备发现
    QTimer *timeoutTimer = new QTimer(this);
    timeoutTimer->setSingleShot(true);
    connect(timeoutTimer, &QTimer::timeout, this, [this]()
            {
        if (lidarDiscoveryActive) {
            logMessage("设备发现超时，未发现设备，停止扫描");
            stopLidarDiscovery();
        } });

    // 立即发送一次，然后每3秒发送一次
    sendLidarBroadcastDiscovery();
    lidarDiscoveryTimer->start(3000);
    timeoutTimer->start(30000);

    lidarDiscoveryActive = true;
    logMessage("设备发现已启动，正在扫描网络中的Livox设备...");
}

void LivoxViewerWindow::stopLidarDiscovery()
{
    if (!lidarDiscoveryActive)
    {
        return;
    }

    logMessage("正在停止设备发现...");

    if (lidarDiscoveryTimer)
    {
        lidarDiscoveryTimer->stop();
        lidarDiscoveryTimer->deleteLater();
        lidarDiscoveryTimer = nullptr;
    }

    if (lidarDiscoverySocket)
    {
        // 先断开信号连接，避免在关闭过程中触发回调
        lidarDiscoverySocket->disconnect();
        lidarDiscoverySocket->close();
        lidarDiscoverySocket->deleteLater();
        lidarDiscoverySocket = nullptr;
    }

    lidarDiscoveryActive = false;
    logMessage("设备发现已停止");
}

void LivoxViewerWindow::sendLidarBroadcastDiscovery()
{
    if (!lidarDiscoverySocket || !lidarDiscoveryActive)
    {
        return;
    }

    // 广播发现命令
    QByteArray discoveryCmd = LidarDiscoveryService::discoveryCommand();

    // 诊断：打印当前 socket 本地地址（便于确认发送接口）
    QHostAddress la = lidarDiscoverySocket->localAddress();
    quint16 lp = lidarDiscoverySocket->localPort();
    logMessage(QString("准备发送广播（socket local=%1:%2）").arg(la.toString()).arg(lp));

#ifdef _WIN32
    // Windows 原有逻辑：直接发送到全局广播地址255.255.255.255:56000
    qint64 sent = lidarDiscoverySocket->writeDatagram(discoveryCmd, QHostAddress::Broadcast, 56000);

    if (sent == -1)
    {
        logMessage("错误: 广播发现命令发送失败: " + lidarDiscoverySocket->errorString());
    }
    else
    {
        static int sendCount = 0;
        sendCount++;
        if (sendCount <= 3)
        {
            logMessage(QString("已发送广播发现命令 (%1 字节, UDP广播到255.255.255.255:56000)").arg(sent));
        }
    }
#else
    // Linux 优化逻辑：使用 SO_BINDTODEVICE 绑定网卡，并严格向 255.255.255.255 发送广播
    qint64 totalSent = 0;

    // 获取底层 socket 描述符
    qintptr sockfd = lidarDiscoverySocket->socketDescriptor();
    if (sockfd != -1) {
        // 获取当前选择的网卡系统名称 (例如 "eth0", "enp3s0")
        QByteArray ifaceName = selectedNetworkInterfaceSysName.toLocal8Bit();

        if (!ifaceName.isEmpty()) {
            // 使用 SO_BINDTODEVICE 将 socket 绑定到特定网卡
            if (setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, ifaceName.constData(), ifaceName.length()) < 0) {
                // 注意: SO_BINDTODEVICE 通常需要 root 权限或 CAP_NET_RAW 能力
                logMessage(QString("警告: Linux 下 SO_BINDTODEVICE 绑定到 %1 失败 (可能需要 sudo)。错误码: %2").arg(selectedNetworkInterfaceSysName).arg(errno));
            } else {
                static int bindLogCount = 0;
                if (bindLogCount == 0) {
                    logMessage(QString("已成功将 Socket 物理绑定到网卡: %1").arg(selectedNetworkInterfaceSysName));
                    bindLogCount++;
                }
            }

            // 确保开启广播权限 (Qt QUdpSocket 通常默认开启，安全起见底层再确认一次)
            int broadcastEnable = 1;
            setsockopt(sockfd, SOL_SOCKET, SO_BROADCAST, &broadcastEnable, sizeof(broadcastEnable));
        } else {
            logMessage("警告: 未获取到所选网卡名称，跳过 SO_BINDTODEVICE 绑定");
        }
    } else {
        logMessage("警告: 无法获取底层的 socket 描述符");
    }

    // 严格发送到全局广播地址 255.255.255.255
    totalSent = lidarDiscoverySocket->writeDatagram(discoveryCmd, QHostAddress::Broadcast, 56000);

    if (totalSent == -1)
    {
        logMessage("错误: 广播发现命令发送失败: " + lidarDiscoverySocket->errorString());
    }
    else
    {
        static int sendCount = 0;
        sendCount++;
        if (sendCount <= 3)
        {
            logMessage(QString("已严格向全局广播 255.255.255.255 发送发现命令 (UDP端口:56000, 字节:%1)").arg(totalSent));
        }
    }
#endif
}

void LivoxViewerWindow::onLidarDiscoveryResponse(const QByteArray &data, const QHostAddress &sender)
{
    try
    {
        // 打印接收到的原始数据包内容
        QString hexData = data.toHex(' ');
        // logMessage(QString("收到UDP数据包，发送者: %1，数据长度: %2 字节").arg(sender.toString()).arg(data.size()));
        // logMessage(QString("原始数据: %1").arg(hexData));

        // 调用详细的包分析函数
        // printPacketDetails(data, sender);

        // 检查起始字节
        if (static_cast<unsigned char>(data[0]) != 0xAA)
        {
            // logMessage(QString("起始字节错误，期望0xAA，实际收到0x%1").arg(QString::number(static_cast<unsigned char>(data[0]), 16).toUpper()));
            return;
        }

        // 检查协议版本
        if (static_cast<unsigned char>(data[1]) != 0x00)
        {
            // logMessage(QString("协议版本错误，期望0x00，实际收到0x%1").arg(QString::number(static_cast<unsigned char>(data[1]), 16).toUpper()));
            return;
        }

        // 解析长度字段 (小端序)
        uint16_t length = (static_cast<unsigned char>(data[3]) << 8) | static_cast<unsigned char>(data[2]);
        // logMessage(QString("数据包长度字段: %1 字节 (小端序)").arg(length));
        if (data.size() < length)
        {
            // logMessage(QString("数据包长度不匹配，长度字段: %1，实际数据: %2").arg(length).arg(data.size()));
            return;
        }

        // 检查cmd_id是否为0x0000 (广播发现)
        uint16_t cmdId = (static_cast<unsigned char>(data[8]) << 8) | static_cast<unsigned char>(data[9]);
        // logMessage(QString("命令ID: 0x%1").arg(QString::number(cmdId, 16).toUpper().rightJustified(4, '0')));
        if (cmdId != 0x0000)
        {
            // logMessage(QString("命令ID不匹配，期望0x0000，实际收到0x%1").arg(QString::number(cmdId, 16).toUpper().rightJustified(4, '0')));
            return;
        }

        // 检查cmd_type是否为0x01 (ACK回复)
        uint8_t cmdType = static_cast<unsigned char>(data[10]);
        // logMessage(QString("命令类型: 0x%1").arg(QString::number(cmdType, 16).toUpper().rightJustified(2, '0')));
        if (cmdType != 0x01)
        {
            // logMessage(QString("命令类型不匹配，期望0x01 (ACK)，实际收到0x%1").arg(QString::number(cmdType, 16).toUpper().rightJustified(2, '0')));
            return;
        }

        // 检查sender_type是否为1 (雷达)
        uint8_t senderType = static_cast<unsigned char>(data[11]);
        // logMessage(QString("发送者类型: 0x%1").arg(QString::number(senderType, 16).toUpper().rightJustified(2, '0')));
        if (senderType != 0x01)
        {
            // logMessage(QString("发送者类型不匹配，期望0x01 (雷达)，实际收到0x%1").arg(QString::number(senderType, 16).toUpper().rightJustified(2, '0')));
            return;
        }

        // 检查data段长度
        if (data.size() < 24 + 24)
        { // 包头24字节 + data段至少24字节
            // logMessage(QString("data段长度不足，需要至少24字节，实际可用%1字节").arg(data.size() - 24));
            return;
        }

        // 解析data段
        int dataOffset = 24; // 跳过包头
        // logMessage(QString("data段起始偏移: %1 字节").arg(dataOffset));

        uint8_t retCode = static_cast<unsigned char>(data[dataOffset]);
        // logMessage(QString("返回码: 0x%1").arg(QString::number(retCode, 16).toUpper().rightJustified(2, '0')));
        if (retCode != 0x00)
        {
            // logMessage(QString("设备发现回复错误码: %1，表示操作失败").arg(retCode));
            return;
        }

        // 解析雷达类型
        uint8_t devType = static_cast<unsigned char>(data[dataOffset + 1]);
        // 记录最近一次发现到的雷达型号，用于后续自动校正配置文件中的 Device type
        lastDiscoveredLidarType = devType;
        hasLastDiscoveredLidarType = true;
        // logMessage(QString("雷达类型: 0x%1").arg(QString::number(devType, 16).toUpper().rightJustified(2, '0')));

        // 解析序列号 (16字节)
        QByteArray serialNumber = data.mid(dataOffset + 2, 16);
        QString serialStr = QString::fromLatin1(serialNumber).trimmed();
        // logMessage(QString("雷达序列号: %1").arg(serialStr));

        // 跳过ret_code, dev_type, serial_number (1+1+16=18字节)
        int ipOffset = dataOffset + 18;
        // logMessage(QString("IP地址偏移: %1 字节").arg(ipOffset));

        // 解析雷达IP地址 (4字节)
        if (ipOffset + 4 > data.size())
        {
            // logMessage(QString("IP地址偏移超出数据包范围，偏移: %1，数据包大小: %2").arg(ipOffset).arg(data.size()));
            return;
        }

        QString deviceIP = QString("%1.%2.%3.%4")
                               .arg(static_cast<unsigned char>(data[ipOffset]))
                               .arg(static_cast<unsigned char>(data[ipOffset + 1]))
                               .arg(static_cast<unsigned char>(data[ipOffset + 2]))
                               .arg(static_cast<unsigned char>(data[ipOffset + 3]));

        // logMessage(QString("解析到雷达IP: %1").arg(deviceIP));

        // 解析控制端口 (2字节, 小端序)
        if (ipOffset + 6 <= data.size())
        {
            uint16_t cmdPort = (static_cast<unsigned char>(data[ipOffset + 5]) << 8) |
                               static_cast<unsigned char>(data[ipOffset + 4]);
            // logMessage(QString("雷达控制端口: %1").arg(cmdPort));
        }

        logMessage(QString("发现雷达: %1 (IP: %2)").arg(sender.toString()).arg(deviceIP));

        // 检查是否需要更新主机IP
        QString currentHostIP = getSelectedHostIP();
        if (!currentHostIP.isEmpty())
        {
            // 检查是否在同一网段
            QHostAddress deviceAddr(deviceIP);
            QHostAddress hostAddr(currentHostIP);

            if (deviceAddr.isNull() || hostAddr.isNull())
            {
                return;
            }

            // 计算网络地址 (假设子网掩码为255.255.255.0)
            quint32 deviceNet = deviceAddr.toIPv4Address() & 0xFFFFFF00;
            quint32 hostNet = hostAddr.toIPv4Address() & 0xFFFFFF00;

            if (deviceIP == currentHostIP) {
                logMessage(QString("检测到设备IP与主机IP完全相同 (%1)，存在地址冲突，必须更新主机IP")
                               .arg(deviceIP));

                QString newHostIP = calculateCompatibleHostIP(deviceIP);
                if (!newHostIP.isEmpty()) {
                    logMessage(QString("建议主机IP: %1 (与设备IP %2 在同一网段)").arg(newHostIP).arg(deviceIP));

                    if (!autoConfigHostIpEnabled) {
                        logMessage("已关闭“自动修改主机网口IP”，请手动修改网口IP后重启程序");
                        logMessage(QString("手动设置步骤: 网口IP设为 %1，子网掩码设为 255.255.255.0").arg(newHostIP));

                        const QString tipIp = newHostIP;
                        QTimer::singleShot(0, this, [this, tipIp]() {
                            if (manualNetworkConfigPromptActive) return;
                            manualNetworkConfigPromptActive = true;
                            QMessageBox::warning(this, "网络配置冲突",
                                                 QString("检测到设备IP与主机IP冲突。\n"
                                                         "请手动修改所选网卡 IPv4 后重启程序。\n\n"
                                                         "建议主机IP: %1 / 掩码: 255.255.255.0").arg(tipIp));
                            manualNetworkConfigPromptActive = false;
                            stopLidarDiscovery();
                        });
                    } else if (updateHostIPForDevice(deviceIP)) {
                        logMessage(QString("主机IP已自动更新为: %1").arg(newHostIP));
                    } else {
                        logMessage("自动更新主机IP失败，请手动修改网口IP后重启程序");
                    }
                }
                return; // 提前退出，避免继续误初始化 SDK
            }
            else if (deviceNet != hostNet)
            {
                logMessage(QString("设备IP %1 与主机IP %2 不在同一网段，需要更新主机IP").arg(deviceIP).arg(currentHostIP));

                // 计算兼容的主机IP
                QString newHostIP = calculateCompatibleHostIP(deviceIP);
                if (!newHostIP.isEmpty())
                {
                    logMessage(QString("建议主机IP: %1 (与设备IP %2 在同一网段)").arg(newHostIP).arg(deviceIP));

                    // 检查是否已经尝试过更新IP，避免无限循环
                    static QString lastAttemptedIP;
                    if (lastAttemptedIP == newHostIP)
                    {
                        logMessage("警告: 已尝试过更新IP为 " + newHostIP + "，避免无限循环");
                        logMessage("请手动检查网络配置或重启程序");
                        stopLidarDiscovery();
                        return;
                    }
                    lastAttemptedIP = newHostIP;

                    if (lidarDiscoverySocket)
                    {
                        lidarDiscoverySocket->close();
                        lidarDiscoverySocket->deleteLater();
                        lidarDiscoverySocket = nullptr;
                        logMessage("已临时关闭 discovery socket 以避免修改IP冲突");
                    }

                    // 尝试自动更新主机IP
                    if (!autoConfigHostIpEnabled)
                    {
                        logMessage("已关闭“自动修改主机网口IP”，将不会自动修改网络配置");
                        logMessage(QString("手动设置步骤: 网口IP设为 %1，子网掩码设为 255.255.255.0").arg(newHostIP));

                        const QString tipIp = newHostIP;
                        QTimer::singleShot(0, this, [this, tipIp]() {
                            if (manualNetworkConfigPromptActive) return;
                            manualNetworkConfigPromptActive = true;
                            QMessageBox::warning(this, "需要手动配置网络",
                                                 QString("设备与主机不在同一网段。\n"
                                                         "请手动设置所选网卡 IPv4 后重启程序。\n\n"
                                                         "建议主机IP: %1 / 掩码: 255.255.255.0").arg(tipIp));
                            manualNetworkConfigPromptActive = false;
                            stopLidarDiscovery();
                        });
                        return;
                    }

                    if (updateHostIPForDevice(deviceIP))
                    {
                        logMessage(QString("主机IP已自动更新为: %1").arg(newHostIP));

                        // 更新 config.json 文件
                        if (updateConfigFileIP(newHostIP))
                        {
                            logMessage("配置文件已更新，准备重新启动程序以应用新的网络配置");

                            // 停止设备发现，避免旧 socket 引发崩溃
                            stopLidarDiscovery();

                            //打印sdk_initialized状态
                            logMessage(QString("sdk_initialized:%1").arg(sdk_initialized));


                            refreshNetworkInterfaces();

                            startLidarDiscovery();
                            // // 提示重启
                            // QMessageBox::warning(this, "重新启动", "准备重新启动应用程序以应用新的网络配置");

                            // logMessage("正在重启程序，请稍候...");

                            // // 启动一个新的进程，运行当前应用
                            // QProcess::startDetached(QApplication::applicationFilePath(), QApplication::arguments());

                            // // 退出当前进程
                            // QApplication::quit();
                            // return;
                        }
                        else
                        {
                            logMessage("配置文件更新失败，请手动检查 config.json");
                        }
                    }
                    else
                    {
                        logMessage("自动更新主机IP失败，请手动设置网口IP后重启程序");
                        logMessage(QString("手动设置步骤: 网口IP设为 %1，子网掩码设为 255.255.255.0").arg(newHostIP));
                    }
                }
            }
            else
            {
                logMessage(QString("设备IP %1 与主机IP %2 在同一网段，无需更新").arg(deviceIP).arg(currentHostIP));

                // 发现设备后，延迟停止设备发现，然后初始化SDK
                logMessage("设备发现完成，准备停止扫描并初始化SDK");

                QTimer *stopTimer = new QTimer(this);
                stopTimer->setSingleShot(true);
                connect(stopTimer, &QTimer::timeout, this, [this, stopTimer]()
                        {
                try {
                    if (lidarDiscoveryActive) {
                        stopLidarDiscovery();
                    }

                    // 延迟初始化SDK，确保设备发现完全停止
                    QTimer* initTimer = new QTimer(this);
                    initTimer->setSingleShot(true);
                    connect(initTimer, &QTimer::timeout, this, [this, initTimer]() {
                        try {
                            logMessage("设备发现已完成，开始初始化SDK...");
                            initializeLivoxSdk();
                            initTimer->deleteLater();
                        } catch (...) {
                            logMessage("初始化SDK时发生异常");
                            initTimer->deleteLater();
                        }
                    });
                    initTimer->start(500);

                    stopTimer->deleteLater();
                } catch (...) {
                    logMessage("停止设备发现时发生异常");
                    stopTimer->deleteLater();
                } });
                stopTimer->start(100);
            }
        }
    }
    catch (const std::exception &e)
    {
        logMessage(QString("设备发现响应处理异常: %1").arg(e.what()));
    }
    catch (...)
    {
        logMessage("设备发现响应处理时发生未知异常");
    }
}

QString LivoxViewerWindow::calculateCompatibleHostIP(const QString &deviceIP)
{
    const QString candidate = NetworkInterfaceService::calculateCompatibleHostIp(deviceIP);
    if (!candidate.isEmpty()) {
        logMessage(QString("Compatible host IP candidate: %1").arg(candidate));
    }
    return candidate;
}
bool LivoxViewerWindow::updateHostIPForDevice(const QString &deviceIP)
{
    if (!autoConfigHostIpEnabled)
    {
        logMessage("已关闭“自动修改主机网口IP”，跳过自动配置");
        return false;
    }

    QString newHostIP = calculateCompatibleHostIP(deviceIP);
    if (newHostIP.isEmpty())
    {
        return false;
    }

    // 只更新“用户选择的网口”的 IP
    // Windows 的 netsh 更偏好使用 humanReadableName（如“以太网”），Linux/mac 更适合使用系统名（如 eth0/enpXsY）
    const QString selectedHumanName = selectedNetworkInterfaceHumanName;
    const QString selectedSysName = selectedNetworkInterfaceSysName;

    if (selectedNetworkIP.isEmpty())
    {
        logMessage("未选择网卡，无法更新主机IP");
        return false;
    }

#ifdef _WIN32
    const QString ifaceNameForNetsh = !selectedHumanName.isEmpty() ? selectedHumanName : selectedSysName;
    if (ifaceNameForNetsh.isEmpty())
    {
        logMessage("未获取到所选网卡名称，无法更新主机IP");
        return false;
    }

    // Windows下尝试使用netsh命令（需要管理员权限）
    logMessage(QString("准备更新所选网口 \"%1\" (%2) 的IP为: %3")
                   .arg(selectedHumanName.isEmpty() ? ifaceNameForNetsh : selectedHumanName)
                   .arg(selectedNetworkIP)
                   .arg(newHostIP));
    // ---- 检查冲突：防止同一网段中已有相同IP ----
    QString basePrefix = newHostIP.section('.', 0, 2); // 例如 "192.168.1"
    QString candidateIP = newHostIP;

    QSet<QString> existingIPs;
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
        for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
            if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol) {
                existingIPs.insert(entry.ip().toString());
            }
        }
    }

    int lastOctet = qMax(newHostIP.section('.',3,3).toInt(), 50); // 从50开始尝试
    while (existingIPs.contains(candidateIP) && lastOctet < 254) {
        lastOctet++;
        candidateIP = QString("%1.%2").arg(basePrefix).arg(lastOctet);
    }

    if (candidateIP != newHostIP) {
        logMessage(QString("检测到IP %1 已被占用，自动调整为可用IP: %2").arg(newHostIP).arg(candidateIP));
        newHostIP = candidateIP;
    }

    logMessage(QString("最终准备设置接口 %1 的IP: %2").arg(ifaceNameForNetsh, newHostIP));
    QProcess process;
    QStringList arguments;
    arguments << "interface" << "ip" << "set" << "address"
              << "name=" + ifaceNameForNetsh
              << "static" << newHostIP << "255.255.255.0";

    process.start("netsh", arguments);
    if (!process.waitForFinished(10000))
    {
        logMessage("设置IP地址超时，请手动设置或使用管理员权限运行程序");
        return false;
    }

    if (process.exitCode() != 0)
    {
        logMessage(QString("设置IP地址失败: %1").arg(QString::fromLocal8Bit(process.readAllStandardError())));
        logMessage("请手动设置网口IP或使用管理员权限运行程序");
        return false;
    }
    #else
        const QString deviceName = selectedSysName; // 传入的网卡名，例如 "ens33"
        const QString newIpWithMask = newHostIP + "/24";

        if (deviceName.isEmpty()) {
            logMessage("错误: 未能获取到有效的网卡系统名称");
            return false;
        }

        logMessage(QString("正在使用 nmcli 修改网卡 %1 的主 IP 为: %2...").arg(deviceName).arg(newHostIP));

        // 1. 自动获取该设备对应的 Connection Name
        // 命令解释：显示活跃连接，过滤出设备名为 ens33 的行，并提取连接名部分
        QProcess getConnProcess;
        QString findConnCmd = QString("nmcli -g DEVICE,NAME connection show --active | grep \"^%1:\" | cut -d: -f2").arg(deviceName);
        getConnProcess.start("sh", QStringList() << "-c" << findConnCmd);
        getConnProcess.waitForFinished(3000);

        QString connectionName = QString::fromLocal8Bit(getConnProcess.readAllStandardOutput()).trimmed();

        // 如果活跃连接里没找到，尝试从所有连接里找
        if (connectionName.isEmpty()) {
            findConnCmd = QString("nmcli -g DEVICE,NAME connection show | grep \"^%1:\" | cut -d: -f2").arg(deviceName);
            getConnProcess.start("sh", QStringList() << "-c" << findConnCmd);
            getConnProcess.waitForFinished(3000);
            connectionName = QString::fromLocal8Bit(getConnProcess.readAllStandardOutput()).trimmed();
        }

        if (connectionName.isEmpty()) {
            logMessage(QString("错误: 无法找到设备 %1 关联的 nmcli 连接配置").arg(deviceName));
            return false;
        }

        logMessage(QString("识别到连接名: [%1], 准备执行持久化修改...").arg(connectionName));

        // 2. 构造 nmcli 修改指令
        // ipv4.method manual: 设为静态IP模式（否则会被DHCP覆盖）
        // ipv4.addresses: 设置主IP
        // connection up: 立即刷新生效
        QString nmCmd = QString(
                            "nmcli connection modify \"%1\" ipv4.addresses %2 ipv4.method manual && "
                            "nmcli connection up \"%1\""
                            ).arg(connectionName).arg(newIpWithMask);

        QProcess process;
        // 使用 pkexec 提权执行复合指令
        process.start("pkexec", QStringList() << "sh" << "-c" << nmCmd);

        if (!process.waitForFinished(45000)) {
            logMessage("提权操作超时或用户取消（修改主IP）");
            return false;
        }

        if (process.exitCode() != 0) {
            QString err = QString::fromLocal8Bit(process.readAllStandardError());
            logMessage("nmcli 修改主IP失败: " + err);
            return false;
        }

        logMessage(QString("成功！网卡 %1 (%2) 已修改为主 IP: %3")
                       .arg(deviceName).arg(connectionName).arg(newHostIP));

        return true;
    #endif

    logMessage(QString("主机IP已更新为: %1").arg(newHostIP));

    // 等待网络配置生效
    QTimer *checkTimer = new QTimer(this);
    checkTimer->setSingleShot(true);
    connect(checkTimer, &QTimer::timeout, this, [this, newHostIP, checkTimer]()
            {
        try {
            QString currentIP = getSelectedHostIP();
            if (!currentIP.isEmpty() && currentIP != newHostIP) {
                logMessage(QString("网络配置可能未完全生效，当前IP: %1，期望IP: %2").arg(currentIP).arg(newHostIP));
            } else {
                logMessage("网络配置已生效");
            }
            checkTimer->deleteLater();
        } catch (...) {
            logMessage("检查网络配置时发生异常");
            checkTimer->deleteLater();
        } });
    checkTimer->start(1000);

    return true;
}

bool LivoxViewerWindow::updateConfigFileIP(const QString &newHostIP)
{
    QString message;
    const bool updated = LidarConfigService::updateResolvedConfigHostIp(newHostIP, &message);
    if (!message.isEmpty()) {
        logMessage(message);
    }
    return updated;
}
bool LivoxViewerWindow::updateConfigFileDeviceTypeIfNeeded(const QString &configPath)
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
/*
void LivoxViewerWindow::printPacketDetails(const QByteArray& data, const QHostAddress& sender)
{
    try {
        logMessage("=== 数据包原始16进制内容 ===");
        logMessage(QString("发送者: %1").arg(sender.toString()));
        logMessage(QString("总长度: %1 字节").arg(data.size()));

    if (data.size() >= 24) {
        // 包头字段 (24字节)
        logMessage("--- 包头 (24字节) ---");
        logMessage(QString("SOF: %1").arg(data[0] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("版本: %1").arg(data[1] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("长度: %1 %2 (小端序)").arg(data[2] & 0xFF, 2, 16, QChar('0')).arg(data[3] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("序列号: %1 %2 %3 %4").arg(data[4] & 0xFF, 2, 16, QChar('0')).arg(data[5] & 0xFF, 2, 16, QChar('0')).arg(data[6] & 0xFF, 2, 16, QChar('0')).arg(data[7] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("命令ID: %1 %2").arg(data[8] & 0xFF, 2, 16, QChar('0')).arg(data[9] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("命令类型: %1").arg(data[10] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("发送者类型: %1").arg(data[11] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("保留: %1 %2 %3 %4 %5 %6").arg(data[12] & 0xFF, 2, 16, QChar('0')).arg(data[13] & 0xFF, 2, 16, QChar('0')).arg(data[14] & 0xFF, 2, 16, QChar('0')).arg(data[15] & 0xFF, 2, 16, QChar('0')).arg(data[16] & 0xFF, 2, 16, QChar('0')).arg(data[17] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("CRC16: %1 %2").arg(data[18] & 0xFF, 2, 16, QChar('0')).arg(data[19] & 0xFF, 2, 16, QChar('0')));
        logMessage(QString("CRC32: %1 %2 %3 %4").arg(data[20] & 0xFF, 2, 16, QChar('0')).arg(data[21] & 0xFF, 2, 16, QChar('0')).arg(data[22] & 0xFF, 2, 16, QChar('0')).arg(data[23] & 0xFF, 2, 16, QChar('0')));
    }

    if (data.size() > 24) {
        // 数据段
        logMessage("--- 数据段 ---");
        QByteArray dataSegment = data.mid(32);
        logMessage(QString("长度: %1 字节").arg(dataSegment.size()));
        logMessage(QString("内容: %1").arg(dataSegment.toHex(' ')));

        if (dataSegment.size() >= 24) {
            logMessage("--- 数据段字段 ---");
            logMessage(QString("返回码: %1").arg(dataSegment[0] & 0xFF, 2, 16, QChar('0')));
            logMessage(QString("雷达类型: %1").arg(dataSegment[1] & 0xFF, 2, 16, QChar('0')));
            logMessage(QString("序列号: %1").arg(dataSegment.mid(2, 16).toHex(' ')));
            logMessage(QString("IP地址: %1").arg(dataSegment.mid(18, 4).toHex(' ')));
            logMessage(QString("控制端口: %1").arg(dataSegment.mid(22, 2).toHex(' ')));
        }
    }

    logMessage("======================");
    } catch (const std::exception& e) {
        logMessage(QString("数据包分析异常: %1").arg(e.what()));
    } catch (...) {
        logMessage("数据包分析时发生未知异常");
    }
}
*/
