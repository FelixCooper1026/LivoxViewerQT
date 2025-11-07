#include "mainwindow.h"
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

// 添加网段检查的辅助函数
bool isInSameSubnet(const QString &hostIP, const QString &currentIP, const QString &subnetMask)
{
    QHostAddress hostAddr(hostIP);
    QHostAddress currentAddr(currentIP);
    QHostAddress maskAddr(subnetMask);

    if (hostAddr.isNull() || currentAddr.isNull() || maskAddr.isNull())
    {
        return false;
    }

    // 计算网络地址
    quint32 hostNet = hostAddr.toIPv4Address() & maskAddr.toIPv4Address();
    quint32 currentNet = currentAddr.toIPv4Address() & maskAddr.toIPv4Address();

    return hostNet == currentNet;
}

// 获取当前主机IP地址（优先获取有线网口IP）
QString getCurrentHostIP()
{
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces())
    {
        if ((iface.flags() & QNetworkInterface::IsUp) &&
            (iface.flags() & QNetworkInterface::IsRunning) &&
            !(iface.flags() & QNetworkInterface::IsLoopBack) &&
            !(iface.flags() & QNetworkInterface::IsPointToPoint))
        {

            // 排除无线网口，只检测有线网口
            QString ifaceName = iface.name().toLower();
            if (ifaceName.contains("wlan") || ifaceName.contains("wifi") ||
                ifaceName.contains("wireless") || ifaceName.contains("802.11"))
            {
                continue;
            }

            for (const QNetworkAddressEntry &entry : iface.addressEntries())
            {
                const QHostAddress &addr = entry.ip();
                if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
                    addr.toString() != "0.0.0.0" &&
                    !addr.toString().startsWith("169.254."))
                {
                    return addr.toString();
                }
            }
        }
    }
    return QString();
}

// 检查是否存在已连接的有线网口设备
static bool hasWiredNetworkDeviceConnected()
{
    const auto interfaces = QNetworkInterface::allInterfaces();

    for (const QNetworkInterface &iface : interfaces)
    {
        auto flags = iface.flags();
        QString name = iface.humanReadableName().toLower();
        QString sysName = iface.name().toLower();

        // 只考虑激活的接口
        if (!(flags & QNetworkInterface::IsUp) ||
            !(flags & QNetworkInterface::IsRunning) ||
            (flags & QNetworkInterface::IsLoopBack) ||
            (flags & QNetworkInterface::IsPointToPoint))
            continue;

        // 排除无线、蓝牙、虚拟网卡
        if (name.contains("wifi") || name.contains("wi-fi") || name.contains("wlan") ||
            name.contains("wireless") || name.contains("蓝牙") ||
            sysName.contains("wifi") || sysName.contains("wireless") || sysName.contains("wlan"))
            continue;

        // Windows: “以太网” / “以太网 2” / “以太网 4” 都属于有线接口
        // Qt 给的 name 通常是 ethernet_xxxxxx
        if (!(sysName.startsWith("ethernet") || name.contains("以太网") || name.contains("ethernet")))
            continue;

        // 检查是否有有效 IPv4
        for (const QNetworkAddressEntry &entry : iface.addressEntries())
        {
            const QHostAddress &addr = entry.ip();
            if (addr.protocol() == QAbstractSocket::IPv4Protocol &&
                addr.toString() != "0.0.0.0" &&
                !addr.toString().startsWith("169.254."))
            {
                qDebug() << "[有线接口检测] 检测到活动接口:"
                         << iface.humanReadableName() << "(" << iface.name() << ")"
                         << "IP:" << addr.toString();
                return true;
            }
        }

        // 即使暂时没有IP，也可以认为有线网口“连接着设备但未获得IP”，可选逻辑：
        if (iface.flags() & QNetworkInterface::IsRunning)
        {
            qDebug() << "[有线接口检测] 检测到物理连接的以太网口:" << iface.humanReadableName();
            return true;
        }
    }

    qDebug() << "[有线接口检测] 未检测到活动的有线网口";
    return false;
}

// 检查配置文件中的host_ip是否与当前主机IP完全一致；当不一致时将详细信息写入 details（若提供）
bool checkConfigFileNetworkCompatibility(const QString &configPath, QString *details = nullptr)
{
    QFile configFile(configPath);
    if (!configFile.open(QIODevice::ReadOnly))
    {
        qDebug() << "无法打开配置文件:" << configPath;
        return false;
    }

    QByteArray configData = configFile.readAll();
    configFile.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(configData, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        qDebug() << "配置文件JSON解析错误:" << parseError.errorString();
        return false;
    }

    QJsonObject configObj = doc.object();

    // 遍历所有设备块（如 HAP、MID360 等）的 host_net_info[*].host_ip
    QString currentHostIP = getCurrentHostIP();
    if (currentHostIP.isEmpty())
    {
        qDebug() << "无法获取当前主机IP地址";
        if (details)
            *details = QString("无法获取当前主机IP地址");
        return false;
    }

    // 收集所有候选 host_ip
    QList<QPair<QString, QString>> deviceIpPairs; // <sectionName, ip>
    for (auto it = configObj.begin(); it != configObj.end(); ++it)
    {
        if (!it.value().isObject())
            continue;
        QString sectionName = it.key();
        QJsonObject deviceObj = it.value().toObject();
        if (!deviceObj.contains("host_net_info") || !deviceObj.value("host_net_info").isArray())
            continue;
        QJsonArray hostInfoArray = deviceObj.value("host_net_info").toArray();
        for (const QJsonValue &v : hostInfoArray)
        {
            if (!v.isObject())
                continue;
            QJsonObject hostInfo = v.toObject();
            if (hostInfo.contains("host_ip") && hostInfo.value("host_ip").isString())
            {
                QString ip = hostInfo.value("host_ip").toString();
                if (!ip.isEmpty())
                    deviceIpPairs.append({sectionName, ip});
            }
        }
    }

    if (deviceIpPairs.isEmpty())
    {
        qDebug() << "配置文件中未找到host_ip字段";
        if (details)
            *details = QString("配置文件中未找到任意设备的 host_ip 字段");
        return false;
    }

    // 要求所有设备 host_ip 都与当前主机IP完全一致
    bool allEqual = true;
    QStringList mismatchDetails;
    for (const auto &pair : deviceIpPairs)
    {
        const QString &section = pair.first;
        const QString &ip = pair.second;
        if (ip != currentHostIP)
        {
            allEqual = false;
            mismatchDetails << QString("[%1:%2]").arg(section, ip);
        }
    }

    if (!allEqual)
    {
        qDebug() << "IP不一致: 以下设备host_ip与当前主机IP不一致:" << mismatchDetails.join(", ")
                 << "; 当前主机IP:" << currentHostIP;
        if (details)
        {
            *details = QString("IP不一致: 以下设备 host_ip 与当前主机IP不一致: %1; 当前主机IP: %2")
                           .arg(mismatchDetails.join(", "))
                           .arg(currentHostIP);
        }
        return false;
    }

    // 全部一致
    qDebug() << "IP检查通过，所有设备 host_ip 与当前主机IP一致:" << currentHostIP;
    if (details)
    {
        *details = QString("IP检查通过，所有设备 host_ip 与当前主机IP一致: %1").arg(currentHostIP);
    }
    return true;
}

void MainWindow::setupLivoxSDK()
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
    if (discoveryActive)
    {
        logMessage("设备发现仍在进行中，等待完成...");
        // 延迟重试，但不要无限等待
        static int retryCount = 0;
        if (retryCount < 10)
        { // 最多等待20秒
            retryCount++;
            QTimer::singleShot(2000, this, &MainWindow::setupLivoxSDK);
            return;
        }
        else
        {
            logMessage("设备发现等待超时，强制继续SDK初始化");
            retryCount = 0;
        }
    }

    // 3) 检查网络状态
    QString currentIP = getCurrentHostIP();
    if (currentIP.isEmpty())
    {
        logMessage("错误: 无法获取当前主机IP，SDK初始化失败");
        return;
    }
    logMessage(QString("当前主机IP: %1，继续SDK初始化...").arg(currentIP));

    // 跨平台的SDK库文件检查
    QStringList possiblePaths;

#ifdef _WIN32
    // Windows路径
    possiblePaths = {
        QDir::currentPath() + "/livox_sdk_qt/lib/livox_lidar_sdk_static.lib",
        QDir::currentPath() + "/../livox_sdk_qt/lib/livox_lidar_sdk_static.lib",
        QDir::currentPath() + "/../../livox_sdk_qt/lib/livox_lidar_sdk_static.lib",
        QApplication::applicationDirPath() + "/livox_sdk_qt/lib/livox_lidar_sdk_static.lib",
        QApplication::applicationDirPath() + "/../livox_sdk_qt/lib/livox_lidar_sdk_static.lib"};
#elif defined(__APPLE__)
    // macOS路径
    possiblePaths = {
        QDir::currentPath() + "/livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QDir::currentPath() + "/../livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QApplication::applicationDirPath() + "/livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QApplication::applicationDirPath() + "/../livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        "/usr/local/lib/liblivox_lidar_sdk_static.a",
        "/opt/livox/lib/liblivox_lidar_sdk_static.a"};
#else
    // Linux路径
    possiblePaths = {
        QDir::currentPath() + "/livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QDir::currentPath() + "/../livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QDir::currentPath() + "/../../livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QApplication::applicationDirPath() + "/livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        QApplication::applicationDirPath() + "/../livox_sdk_qt/lib/liblivox_lidar_sdk_static.a",
        // 系统路径
        "/usr/local/lib/liblivox_lidar_sdk_static.a",
        "/usr/lib/liblivox_lidar_sdk_static.a",
        "/usr/lib/x86_64-linux-gnu/liblivox_lidar_sdk_static.a",
        "/opt/livox/lib/liblivox_lidar_sdk_static.a",
        // 动态库路径（如果有）
        "/usr/local/lib/liblivox_lidar_sdk.so",
        "/usr/lib/liblivox_lidar_sdk.so",
        "/usr/lib/x86_64-linux-gnu/liblivox_lidar_sdk.so"};
#endif

    QString foundPath;
    for (const QString &path : possiblePaths)
    {
        if (QFile::exists(path))
        {
            foundPath = path;
            break;
        }
    }

    if (foundPath.isEmpty())
    {
        return;
    }

    // 查找配置文件（仅支持 config.json）
    QStringList configPaths = {
        QDir::currentPath() + "/config.json",
        QApplication::applicationDirPath() + "/config.json",
        QApplication::applicationDirPath() + "/../config.json"};

    QString configPath;
    for (const QString &path : configPaths)
    {
        if (QFile::exists(path))
        {
            configPath = path;
            break;
        }
    }

    if (configPath.isEmpty())
    {
        // 未找到 config.json，提示并弹出配置向导
        logMessage("未找到配置文件 config.json，将打开配置向导");
        if (!runConfigGeneratorDialog())
        {
            logMessage("已取消生成配置文件，SDK 初始化终止");
            return;
        }
        // 生成后再次查找
        for (const QString &path : configPaths)
        {
            if (QFile::exists(path))
            {
                configPath = path;
                break;
            }
        }
        if (configPath.isEmpty())
        {
            logMessage("错误: 仍未找到 config.json");
            return;
        }
    }

    logMessage("找到配置文件: " + configPath);

    // 2) 有线网口已连接设备：检查配置文件 IP 匹配
    logMessage("开始检查配置文件 IP 匹配...");
    {
        QString netCheckDetails;
        bool ok = checkConfigFileNetworkCompatibility(configPath, &netCheckDetails);
        if (!ok)
        {
            if (!netCheckDetails.isEmpty())
                logMessage(netCheckDetails);
            else
                logMessage("配置文件中的host_ip与当前主机IP不一致");
        }
        else
        {
            if (!netCheckDetails.isEmpty())
                logMessage(netCheckDetails);
        }
        if (!ok)
        {
            logMessage("将打开配置向导重新生成配置文件");

            if (!runConfigGeneratorDialog())
            {
                logMessage("已取消重新生成配置文件，SDK 初始化终止");
                return;
            }

            // 重新生成后再次查找配置文件
            QString regeneratedConfigPath;
            for (const QString &path : configPaths)
            {
                if (QFile::exists(path))
                {
                    regeneratedConfigPath = path;
                    break;
                }
            }
            if (regeneratedConfigPath.isEmpty())
            {
                logMessage("错误: 重新生成配置文件后仍未找到");
                return;
            }

            configPath = regeneratedConfigPath;
            logMessage("使用新生成的配置文件，继续初始化 SDK: " + configPath);
            // 不再阻断，直接继续后续 SDK 初始化流程
        }
    }

    try
    {
        if (!LivoxLidarSdkInit(configPath.toStdString().c_str()))
        {
            logMessage("错误: Livox SDK 初始化失败");
            return;
        }

        sdk_initialized = true;
        logMessage("Livox SDK 初始化成功");
        // 获取并打印 SDK 版本信息
        LivoxLidarSdkVer sdkVersion;
        GetLivoxLidarSdkVer(&sdkVersion);
        logMessage(QString("Livox SDK 版本: v%1.%2.%3")
                       .arg(sdkVersion.major)
                       .arg(sdkVersion.minor)
                       .arg(sdkVersion.patch));

        // 设置回调函数
        SetLivoxLidarInfoChangeCallback(onDeviceInfoChange, this);
        SetLivoxLidarPointCloudCallBack(onPointCloudData, this);
        SetLivoxLidarImuDataCallback(onImuData, this);
        SetLivoxLidarInfoCallback(onStatusInfo, this);

        sdk_started = true;
        pointCloudCallbackEnabled = true; // 自动开始采样
        statusLabelBar->setText("已连接 - 采样中");

        QTimer::singleShot(5000, this, [this]()
                           {
            if (devices.isEmpty()) {
            } else {
            } });
    }
    catch (...)
    {
        return;
    }
}

void MainWindow::cleanupLivoxSDK()
{
    if (!sdk_started && !sdk_initialized)
    {
        return;
    }

    // 标记关闭中：回调进入时直接返回（在回调开头检查）
    shutting_down = true;

    // 先解除所有回调注册，防止 Uninit 过程中回调进入
    SetLivoxLidarInfoChangeCallback(nullptr, nullptr);
    SetLivoxLidarPointCloudCallBack(nullptr, nullptr);
    SetLivoxLidarImuDataCallback(nullptr, nullptr);
    SetLivoxLidarInfoCallback(nullptr, nullptr);

    // 清空UI侧设备状态
    {
        QMutexLocker locker(&deviceMutex);
        devices.clear();
    }
    if (deviceList)
        deviceList->clear();
    currentDevice = nullptr;

    // 最后调用 Uninit
    LivoxLidarSdkUninit();

    sdk_started = false;
    sdk_initialized = false;
    shutting_down = false; // 退出完成
    logMessage("Livox SDK 已清理");
}

// 设备发现相关方法实现

void MainWindow::startDeviceDiscovery()
{
    if (discoveryActive)
    {
        return;
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
                    startDeviceDiscovery(); // 重新进入发现流程
                }
                else
                {
                    logMessage("等待有线网口连接中...");
                }
            });
            waitTimer->start();
        }

        return; // 暂时退出，等待下一轮检测
    }

    // 获取有线网口的 IPv4（必须存在）
    QString hwIp = getCurrentHostIP();

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
                        QString refreshedIP = getCurrentHostIP();
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
                        hwIp = getCurrentHostIP();
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
    discoverySocket = new QUdpSocket(this);

    // 尝试绑定到有线 IP 的 56000 端口，重试10次，每次间隔1秒
    int retryCount = 5;
    bool bindOk = false;
    for (int i = 0; i < retryCount; ++i)
    {
        bindOk = discoverySocket->bind(QHostAddress(hwIp), 56000,
                                       QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
        if (bindOk)
        {
            break;
        }
        else
        {
            logMessage(QString("警告: 绑定到 %1:56000 失败 (第%2次重试): %3")
                           .arg(hwIp)
                           .arg(i + 1)
                           .arg(discoverySocket->errorString()));
            // 等待1秒后重试
            QThread::sleep(1);
        }
    }
    if (!bindOk)
    {
        logMessage(QString("警告: 绑定到 %1:56000 失败: %2")
                       .arg(hwIp)
                       .arg(discoverySocket->errorString()));
        // 最小回退：尝试绑定到 AnyIPv4
        if (!discoverySocket->bind(QHostAddress::AnyIPv4, 56000,
                                   QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
        {
            logMessage("警告: 回退绑定到 AnyIPv4 仍失败，无法启动设备发现");
            delete discoverySocket;
            discoverySocket = nullptr;
            return;
        }
        else
        {
            logMessage("已回退绑定到 AnyIPv4:56000（注意：广播可能不会从有线接口发出）");
        }
    }
    else
    {
        logMessage(QString("已绑定 discovery socket 到 %1:56000（将使用该接口发送广播）")
                       .arg(hwIp));
    }

    // 连接readyRead信号
    connect(discoverySocket, &QUdpSocket::readyRead, this, [this]() {
        while (discoverySocket->hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(discoverySocket->pendingDatagramSize());
            QHostAddress sender;
            quint16 senderPort = 0;
            discoverySocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            QString senderIP = sender.toString();
            QString localIP = discoverySocket->localAddress().toString();

            // === Step 1: 构建本机所有IPv4地址集合 ===
            static QSet<QString> localIPv4Set;
            if (localIPv4Set.isEmpty()) {
                for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces()) {
                    if (!(iface.flags() & QNetworkInterface::IsUp) ||
                        !(iface.flags() & QNetworkInterface::IsRunning) ||
                        (iface.flags() & QNetworkInterface::IsLoopBack))
                        continue;
                    for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
                        if (entry.ip().protocol() == QAbstractSocket::IPv4Protocol)
                            localIPv4Set.insert(entry.ip().toString());
                    }
                }
            }

            // === Step 2: 判断是否“来自本机的包” ===
            bool isTrulyLocal = localIPv4Set.contains(senderIP);

            // === Step 3: 判断当前Socket是否绑定在特定接口上 ===
            bool boundToValidInterface = !(localIP == "0.0.0.0" ||
                                           localIP.startsWith("169.254."));

            // === Step 4: 仅当Socket绑定在特定接口上，且包确实来自本机，才忽略 ===
            if (boundToValidInterface && isTrulyLocal) {
                logMessage(QString("忽略来自本机接口的数据包: %1 (绑定接口: %2)")
                               .arg(senderIP)
                               .arg(localIP));
                continue;
            }

            // === Step 5: 额外防护：当主机与设备IP完全一致时，禁止忽略 ===
            QString currentHostIP = getCurrentHostIP();
            if (senderIP == currentHostIP) {
                logMessage(QString("警告: 收到与主机相同IP(%1)的UDP包，可能是雷达设备冲突，仍尝试解析").arg(senderIP));
                // 不continue，继续解析
            }

            // === Step 6: 正常解析 ===
            onDeviceDiscoveryResponse(datagram, sender);
        }
    });


    // 创建定时器，定期发送广播发现命令
    discoveryTimer = new QTimer(this);
    connect(discoveryTimer, &QTimer::timeout, this, &MainWindow::sendBroadcastDiscovery);

    // 创建超时定时器，30秒后自动停止设备发现
    QTimer *timeoutTimer = new QTimer(this);
    timeoutTimer->setSingleShot(true);
    connect(timeoutTimer, &QTimer::timeout, this, [this]()
            {
        if (discoveryActive) {
            logMessage("设备发现超时，未发现设备，停止扫描");
            stopDeviceDiscovery();
        } });

    // 立即发送一次，然后每3秒发送一次
    sendBroadcastDiscovery();
    discoveryTimer->start(3000);
    timeoutTimer->start(30000);

    discoveryActive = true;
    logMessage("设备发现已启动，正在扫描网络中的Livox设备...");
}

void MainWindow::stopDeviceDiscovery()
{
    if (!discoveryActive)
    {
        return;
    }

    logMessage("正在停止设备发现...");

    if (discoveryTimer)
    {
        discoveryTimer->stop();
        discoveryTimer->deleteLater();
        discoveryTimer = nullptr;
    }

    if (discoverySocket)
    {
        // 先断开信号连接，避免在关闭过程中触发回调
        discoverySocket->disconnect();
        discoverySocket->close();
        discoverySocket->deleteLater();
        discoverySocket = nullptr;
    }

    discoveryActive = false;
    logMessage("设备发现已停止");
}

void MainWindow::sendBroadcastDiscovery()
{
    if (!discoverySocket || !discoveryActive)
    {
        return;
    }

    // 广播发现命令
    QByteArray discoveryCmd = QByteArray::fromHex("aa00180002000000000000000000000000000a9200000000");

    // 诊断：打印当前 socket 本地地址（便于确认发送接口）
    QHostAddress la = discoverySocket->localAddress();
    quint16 lp = discoverySocket->localPort();
    logMessage(QString("准备发送广播（socket local=%1:%2）").arg(la.toString()).arg(lp));

    // 发送到广播地址255.255.255.255:56000
    qint64 sent = discoverySocket->writeDatagram(discoveryCmd, QHostAddress::Broadcast, 56000);

    if (sent == -1)
    {
        logMessage("错误: 广播发现命令发送失败: " + discoverySocket->errorString());
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
}

void MainWindow::onDeviceDiscoveryResponse(const QByteArray &data, const QHostAddress &sender)
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
        QString currentHostIP = getCurrentHostIP();
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

                    if (updateHostIPForDevice(deviceIP)) {
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
                        stopDeviceDiscovery();
                        return;
                    }
                    lastAttemptedIP = newHostIP;

                    if (discoverySocket)
                    {
                        discoverySocket->close();
                        discoverySocket->deleteLater();
                        discoverySocket = nullptr;
                        logMessage("已临时关闭 discovery socket 以避免修改IP冲突");
                    }

                    // 尝试自动更新主机IP
                    if (updateHostIPForDevice(deviceIP))
                    {
                        logMessage(QString("主机IP已自动更新为: %1").arg(newHostIP));

                        // 更新 config.json 文件
                        if (updateConfigFileIP(newHostIP))
                        {
                            logMessage("配置文件已更新，准备重新启动程序以应用新的网络配置");

                            // 停止设备发现，避免旧 socket 引发崩溃
                            stopDeviceDiscovery();

                            // 提示重启
                            QMessageBox::warning(this, "重新启动", "准备重新启动应用程序以应用新的网络配置");

                            logMessage("正在重启程序，请稍候...");

                            // 启动一个新的进程，运行当前应用
                            QProcess::startDetached(QApplication::applicationFilePath(), QApplication::arguments());

                            // 退出当前进程
                            QApplication::quit();
                            return;
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
                    if (discoveryActive) {
                        stopDeviceDiscovery();
                    }

                    // 延迟初始化SDK，确保设备发现完全停止
                    QTimer* initTimer = new QTimer(this);
                    initTimer->setSingleShot(true);
                    connect(initTimer, &QTimer::timeout, this, [this, initTimer]() {
                        try {
                            logMessage("设备发现已完成，开始初始化SDK...");
                            setupLivoxSDK();
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

QString MainWindow::calculateCompatibleHostIP(const QString &deviceIP)
{
    QHostAddress deviceAddr(deviceIP);
    if (deviceAddr.isNull())
    {
        return QString();
    }

    quint32 deviceIPInt = deviceAddr.toIPv4Address();
    quint32 networkPart = deviceIPInt & 0xFFFFFF00; // 255.255.255.0 掩码

    // 遍历可用主机地址 2~254，跳过设备自身
    for (int host = 2; host <= 254; ++host)
    {
        if (host == (deviceIPInt & 0xFF))
        {
            continue; // 跳过设备本身
        }

        quint32 candidateIP = networkPart | host;
        QString candidateIPStr = QHostAddress(candidateIP).toString();

        // TODO: 如果需要，ping 测试 candidateIP 是否被占用
        logMessage(QString("尝试兼容主机IP: %1").arg(candidateIPStr));
        return candidateIPStr; // 当前简化处理，直接返回第一个可选IP
    }

    return QString(); // 没有可用IP
}

bool MainWindow::updateHostIPForDevice(const QString &deviceIP)
{
    QString newHostIP = calculateCompatibleHostIP(deviceIP);
    if (newHostIP.isEmpty())
    {
        return false;
    }

    // 获取有线网口名称
    QString wiredInterfaceName;
    for (const QNetworkInterface &iface : QNetworkInterface::allInterfaces())
    {
        if ((iface.flags() & QNetworkInterface::IsUp) &&
            (iface.flags() & QNetworkInterface::IsRunning) &&
            !(iface.flags() & QNetworkInterface::IsLoopBack) &&
            !(iface.flags() & QNetworkInterface::IsPointToPoint))
        {

            QString ifaceName = iface.name().toLower();
            if (ifaceName.contains("wlan") || ifaceName.contains("wifi") ||
                ifaceName.contains("wireless") || ifaceName.contains("802.11"))
            {
                continue;
            }

            wiredInterfaceName = iface.name();
            break;
        }
    }

    if (wiredInterfaceName.isEmpty())
    {
        logMessage("未找到有线网口");
        return false;
    }

#ifdef _WIN32
    // Windows下尝试使用netsh命令（需要管理员权限）
    logMessage(QString("准备更新有线网口 %1 的IP为: %2").arg(wiredInterfaceName).arg(newHostIP));
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

    logMessage(QString("最终准备设置接口 %1 的IP: %2").arg(wiredInterfaceName, newHostIP));
    QProcess process;
    QStringList arguments;
    arguments << "interface" << "ip" << "set" << "address"
              << "name=" + wiredInterfaceName
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
    // Linux/macOS下使用ip命令
    QProcess process;
    QStringList arguments;
    arguments << "addr" << "add" << newHostIP + "/24" << "dev" << wiredInterfaceName;

    process.start("ip", arguments);
    if (!process.waitForFinished(10000))
    {
        logMessage("设置IP地址超时");
        return false;
    }

    if (process.exitCode() != 0)
    {
        logMessage(QString("设置IP地址失败: %1").arg(QString::fromLocal8Bit(process.readAllStandardError())));
        return false;
    }
#endif

    logMessage(QString("主机IP已更新为: %1").arg(newHostIP));

    // 等待网络配置生效
    QTimer *checkTimer = new QTimer(this);
    checkTimer->setSingleShot(true);
    connect(checkTimer, &QTimer::timeout, this, [this, newHostIP, checkTimer]()
            {
        try {
            QString currentIP = getCurrentHostIP();
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

bool MainWindow::updateConfigFileIP(const QString &newHostIP)
{
    // 查找配置文件
    QStringList configPaths = {
        QDir::currentPath() + "/config.json",
        QApplication::applicationDirPath() + "/config.json",
        QApplication::applicationDirPath() + "/../config.json"};

    QString configPath;
    for (const QString &path : configPaths)
    {
        if (QFile::exists(path))
        {
            configPath = path;
            break;
        }
    }

    if (configPath.isEmpty())
    {
        logMessage("未找到配置文件");
        return false;
    }

    // 读取配置文件
    QFile configFile(configPath);
    if (!configFile.open(QIODevice::ReadOnly))
    {
        logMessage("无法打开配置文件");
        return false;
    }

    QByteArray configData = configFile.readAll();
    configFile.close();

    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(configData, &parseError);
    if (parseError.error != QJsonParseError::NoError)
    {
        logMessage("配置文件JSON解析错误");
        return false;
    }

    QJsonObject configObj = doc.object();
    bool updated = false;

    // 更新所有设备的host_ip
    for (auto it = configObj.begin(); it != configObj.end(); ++it)
    {
        if (!it.value().isObject())
            continue;

        QJsonObject deviceObj = it.value().toObject();
        if (!deviceObj.contains("host_net_info") || !deviceObj.value("host_net_info").isArray())
            continue;

        QJsonArray hostInfoArray = deviceObj.value("host_net_info").toArray();
        for (int i = 0; i < hostInfoArray.size(); ++i)
        {
            QJsonObject hostInfo = hostInfoArray[i].toObject();
            if (hostInfo.contains("host_ip"))
            {
                hostInfo["host_ip"] = newHostIP;
                hostInfoArray[i] = hostInfo;
                updated = true;
            }
        }
        deviceObj["host_net_info"] = hostInfoArray;
        configObj[it.key()] = deviceObj;
    }

    if (!updated)
    {
        logMessage("配置文件中未找到host_ip字段");
        return false;
    }

    // 写回配置文件
    QJsonDocument newDoc(configObj);
    if (!configFile.open(QIODevice::WriteOnly))
    {
        logMessage("无法写入配置文件");
        return false;
    }

    configFile.write(newDoc.toJson());
    configFile.close();

    logMessage(QString("配置文件已更新，所有host_ip设置为: %1").arg(newHostIP));
    return true;
}

// void MainWindow::printPacketDetails(const QByteArray& data, const QHostAddress& sender)
// {
//     try {
//         logMessage("=== 数据包原始16进制内容 ===");
//         logMessage(QString("发送者: %1").arg(sender.toString()));
//         logMessage(QString("总长度: %1 字节").arg(data.size()));

//     if (data.size() >= 24) {
//         // 包头字段 (24字节)
//         logMessage("--- 包头 (24字节) ---");
//         logMessage(QString("SOF: %1").arg(data[0] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("版本: %1").arg(data[1] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("长度: %1 %2 (小端序)").arg(data[2] & 0xFF, 2, 16, QChar('0')).arg(data[3] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("序列号: %1 %2 %3 %4").arg(data[4] & 0xFF, 2, 16, QChar('0')).arg(data[5] & 0xFF, 2, 16, QChar('0')).arg(data[6] & 0xFF, 2, 16, QChar('0')).arg(data[7] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("命令ID: %1 %2").arg(data[8] & 0xFF, 2, 16, QChar('0')).arg(data[9] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("命令类型: %1").arg(data[10] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("发送者类型: %1").arg(data[11] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("保留: %1 %2 %3 %4 %5 %6").arg(data[12] & 0xFF, 2, 16, QChar('0')).arg(data[13] & 0xFF, 2, 16, QChar('0')).arg(data[14] & 0xFF, 2, 16, QChar('0')).arg(data[15] & 0xFF, 2, 16, QChar('0')).arg(data[16] & 0xFF, 2, 16, QChar('0')).arg(data[17] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("CRC16: %1 %2").arg(data[18] & 0xFF, 2, 16, QChar('0')).arg(data[19] & 0xFF, 2, 16, QChar('0')));
//         logMessage(QString("CRC32: %1 %2 %3 %4").arg(data[20] & 0xFF, 2, 16, QChar('0')).arg(data[21] & 0xFF, 2, 16, QChar('0')).arg(data[22] & 0xFF, 2, 16, QChar('0')).arg(data[23] & 0xFF, 2, 16, QChar('0')));
//     }

//     if (data.size() > 24) {
//         // 数据段
//         logMessage("--- 数据段 ---");
//         QByteArray dataSegment = data.mid(32);
//         logMessage(QString("长度: %1 字节").arg(dataSegment.size()));
//         logMessage(QString("内容: %1").arg(dataSegment.toHex(' ')));

//         if (dataSegment.size() >= 24) {
//             logMessage("--- 数据段字段 ---");
//             logMessage(QString("返回码: %1").arg(dataSegment[0] & 0xFF, 2, 16, QChar('0')));
//             logMessage(QString("雷达类型: %1").arg(dataSegment[1] & 0xFF, 2, 16, QChar('0')));
//             logMessage(QString("序列号: %1").arg(dataSegment.mid(2, 16).toHex(' ')));
//             logMessage(QString("IP地址: %1").arg(dataSegment.mid(18, 4).toHex(' ')));
//             logMessage(QString("控制端口: %1").arg(dataSegment.mid(22, 2).toHex(' ')));
//         }
//     }

//     logMessage("======================");
//     } catch (const std::exception& e) {
//         logMessage(QString("数据包分析异常: %1").arg(e.what()));
//     } catch (...) {
//         logMessage("数据包分析时发生未知异常");
//     }
// }
