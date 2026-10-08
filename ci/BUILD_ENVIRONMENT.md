# 固定 CI 构建环境

日常 `Build packages` 从 `ci/environment-version` 指定的 Release 获取构建环境，不依赖 Actions 缓存，也不在制包时安装系统包或编译 Qt、Boost、GTSAM。

| 平台 | 固定资产 | 内容 |
| --- | --- | --- |
| Windows | 环境 Release 中的 `windows-env.7z` | Qt 6.8.3、Release 第三方库、Inno Setup |
| Linux | GHCR 镜像；Release 中的 `linux-image.txt` 记录镜像 digest | Ubuntu 20.04、GCC 10、Qt 6.8.3、OpenSSL 1.1、系统开发包、Release 第三方库、AppImage 工具 |

Release 资产和 GHCR 镜像不受 Actions 缓存的闲置清理规则影响。请保留正在使用的环境 Release 和镜像，不要对它们配置删除规则。镜像按 digest 拉取，因此更改同名镜像标签不会改变已发布的环境。

## 日常制包

运行 `Build packages` 即可。Windows 解压环境到 `C:\livox-build-env`，Linux 在预制镜像中构建。Linux 的 DEB 和 AppImage 共用同一编译目录，主程序只需编译一次。

手动运行只上传应用制包产物；应用版本标签的发布行为与原工作流一致。环境 Release 使用 `ci-env-*` 标签并标记为预发布，不会成为应用更新接口的最新正式版本。

## 更新固定环境

1. 修改需要升级的依赖版本或 `ci/linux/Dockerfile`，将 `ci/environment-version` 改为新的环境版本，例如 `ci-env-v2`。
2. 推送后手动运行 `Build fixed CI environment`。它会独立构建 Windows 依赖和 Linux 镜像，全部完成后发布环境 Release。
3. 运行一次 `Build packages` 验证四种应用安装包。后续日常 CI 继续复用此环境。

首次创建环境仍需要完整编译依赖。环境工作流失败时使用 GitHub 的 `Re-run failed jobs`，保留已经创建的草稿 Release 和成功的任务。已发布环境不覆盖；后续升级使用新版本号。

## 路径与工具链

预编译库的 CMake 配置引用固定安装路径：Windows 使用 `C:\livox-build-env\third-party`，Linux 使用 `/opt/livox/third-party`。工作流通过 `LIVOX_THIRD_PARTY_ROOT` 将路径传给项目；本地构建默认仍使用仓库中的 `third-party`。

Windows 使用 GitHub 托管的 `windows-2022` 和 MSVC 2022。该 runner 标签固定系统系列，GitHub 仍会更新补丁和工具链；它不是完整冻结的 Windows 虚拟机。Qt、第三方库和 Inno Setup 已存为资产。若未来更换 MSVC 主工具链，应同时建立新的环境版本。

Linux 镜像固定了实际系统库和编译器文件。维护者需要升级系统或依赖时，主动构建新环境。日常运行耗时主要为资产下载、镜像拉取及项目源码编译，具体耗时取决于 GitHub runner 和网络。
