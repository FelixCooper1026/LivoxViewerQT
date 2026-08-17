param(
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$thirdPartyRoot = Join-Path $repoRoot "third-party"
$downloadRoot = Join-Path $thirdPartyRoot ".downloads"

New-Item -ItemType Directory -Force -Path $thirdPartyRoot | Out-Null
New-Item -ItemType Directory -Force -Path $downloadRoot | Out-Null

$vswhere = Join-Path ${env:ProgramFiles(x86)} "Microsoft Visual Studio\Installer\vswhere.exe"
$vsInstall = & $vswhere `
    -latest `
    -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -property installationPath
$vsVersion = & $vswhere `
    -latest `
    -products * `
    -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 `
    -property installationVersion
if (-not $vsInstall) {
    throw "Visual Studio with the MSVC x64 toolset was not found"
}
$vcvarsPath = Join-Path $vsInstall "VC\Auxiliary\Build\vcvars64.bat"
$msvcToolsRoot = Join-Path $vsInstall "VC\Tools\MSVC"
$msvcTools = Get-ChildItem -LiteralPath $msvcToolsRoot -Directory |
    Sort-Object { [version]$_.Name } -Descending |
    Select-Object -First 1
$clPath = Join-Path $msvcTools.FullName "bin\Hostx64\x64\cl.exe"

function Install-ZipDependency {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Url,
        [Parameter(Mandatory = $true)][string]$ArchiveName,
        [Parameter(Mandatory = $true)][string]$InstallDirName,
        [Parameter(Mandatory = $true)][string]$RequiredFile,
        [string]$SourceSubdirectory = ""
    )

    $installDir = Join-Path $thirdPartyRoot $InstallDirName
    $requiredPath = Join-Path $installDir $RequiredFile

    if ((Test-Path -LiteralPath $requiredPath) -and -not $Force) {
        Write-Host "$Name already installed: $installDir"
        return
    }

    if ((Test-Path -LiteralPath $installDir) -and $Force) {
        Remove-Item -LiteralPath $installDir -Recurse -Force
    }

    $archivePath = Join-Path $downloadRoot $ArchiveName
    $extractRoot = Join-Path $downloadRoot ([IO.Path]::GetFileNameWithoutExtension($ArchiveName))

    if (-not (Test-Path -LiteralPath $archivePath)) {
        Write-Host "Downloading $Name..."
        Invoke-WebRequest -Uri $Url -OutFile $archivePath
    }

    if (Test-Path -LiteralPath $extractRoot) {
        Remove-Item -LiteralPath $extractRoot -Recurse -Force
    }

    Write-Host "Extracting $Name..."
    New-Item -ItemType Directory -Force -Path $extractRoot | Out-Null
    & tar.exe -xf $archivePath -C $extractRoot
    if ($LASTEXITCODE -ne 0) {
        throw "$Name extraction failed with exit code $LASTEXITCODE"
    }

    $sourceRoot = Get-ChildItem -LiteralPath $extractRoot -Directory | Select-Object -First 1
    if ($null -eq $sourceRoot) {
        throw "Archive did not contain an extracted source directory: $archivePath"
    }
    if ($SourceSubdirectory) {
        $sourceRoot = Get-Item -LiteralPath (Join-Path $sourceRoot.FullName $SourceSubdirectory)
    }

    New-Item -ItemType Directory -Force -Path $installDir | Out-Null
    Get-ChildItem -LiteralPath $sourceRoot.FullName -Force | Move-Item -Destination $installDir -Force

    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "$Name installation failed; missing $requiredPath"
    }

    Write-Host "$Name installed: $installDir"
}

function Install-OpenCvPrebuilt {
    $name = "OpenCV 4.13.0"
    $installDir = Join-Path $thirdPartyRoot "opencv-4.13.0"
    $requiredPath = Join-Path $installDir "build\OpenCVConfig.cmake"
    if ((Test-Path -LiteralPath $requiredPath) -and -not $Force) {
        Write-Host "$name already installed: $installDir"
        return
    }
    if ((Test-Path -LiteralPath $installDir) -and $Force) {
        Remove-Item -LiteralPath $installDir -Recurse -Force
    }

    $archivePath = Join-Path $downloadRoot "opencv-4.13.0-windows.exe"
    $expectedSha256 = "f0e98c302464d6860777a7015065e11b9b271b5394e6ba92663f0cf1fc303f2c"
    if (-not (Test-Path -LiteralPath $archivePath)) {
        Write-Host "Downloading $name..."
        Invoke-WebRequest -Uri "https://github.com/opencv/opencv/releases/download/4.13.0/opencv-4.13.0-windows.exe" -OutFile $archivePath
    }
    $actualSha256 = (Get-FileHash -LiteralPath $archivePath -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualSha256 -ne $expectedSha256) {
        throw "$name checksum mismatch: $actualSha256"
    }

    $extractRoot = Join-Path $downloadRoot "opencv-4.13.0-windows"
    if (Test-Path -LiteralPath $extractRoot) {
        Remove-Item -LiteralPath $extractRoot -Recurse -Force
    }
    New-Item -ItemType Directory -Force -Path $extractRoot | Out-Null
    Write-Host "Extracting $name..."
    & $archivePath "-o$extractRoot" -y | Out-Null
    if ($LASTEXITCODE -ne 0) {
        throw "$name extraction failed with exit code $LASTEXITCODE"
    }
    $sourceRoot = Join-Path $extractRoot "opencv"
    if (-not (Test-Path -LiteralPath $sourceRoot)) {
        throw "$name archive did not contain the expected opencv directory"
    }
    Move-Item -LiteralPath $sourceRoot -Destination $installDir
    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "$name installation failed; missing $requiredPath"
    }
    Write-Host "$name installed: $installDir"
}

Install-ZipDependency `
    -Name "Eigen 3.4.0" `
    -Url "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip" `
    -ArchiveName "eigen-3.4.0.zip" `
    -InstallDirName "eigen-3.4.0" `
    -RequiredFile "Eigen/Core"

Install-ZipDependency `
    -Name "MCAP C++ 2.1.3" `
    -Url "https://codeload.github.com/foxglove/mcap/zip/refs/tags/releases/cpp/v2.1.3" `
    -ArchiveName "mcap-cpp-2.1.3.zip" `
    -InstallDirName "mcap-cpp-2.1.3" `
    -RequiredFile "include/mcap/reader.hpp" `
    -SourceSubdirectory "cpp/mcap"

Install-ZipDependency `
    -Name "Zstd 1.5.7" `
    -Url "https://codeload.github.com/facebook/zstd/zip/refs/tags/v1.5.7" `
    -ArchiveName "zstd-1.5.7.zip" `
    -InstallDirName "zstd-1.5.7" `
    -RequiredFile "build/cmake/CMakeLists.txt"

Install-OpenCvPrebuilt

$eigenRoot = Join-Path $thirdPartyRoot "eigen-3.4.0"
$eigenConfig = Join-Path $eigenRoot "share\eigen3\cmake"
New-Item -ItemType Directory -Force -Path $eigenConfig | Out-Null
$eigenRootCMake = $eigenRoot.Replace('\', '/')
$eigenPackageConfig = @"
set(EIGEN3_FOUND TRUE)
set(EIGEN3_VERSION "3.4.0")
set(EIGEN3_VERSION_STRING "3.4.0")
set(EIGEN3_INCLUDE_DIR "$eigenRootCMake")
set(EIGEN3_INCLUDE_DIRS "$eigenRootCMake")
if(NOT TARGET Eigen3::Eigen)
    add_library(Eigen3::Eigen INTERFACE IMPORTED)
    set_target_properties(Eigen3::Eigen PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "$eigenRootCMake")
endif()
"@
Set-Content -LiteralPath (Join-Path $eigenConfig "Eigen3Config.cmake") `
    -Value $eigenPackageConfig `
    -Encoding Ascii

Install-ZipDependency `
    -Name "Boost 1.82.0" `
    -Url "https://archives.boost.io/release/1.82.0/source/boost_1_82_0.zip" `
    -ArchiveName "boost-1.82.0.zip" `
    -InstallDirName "boost-1.82.0" `
    -RequiredFile "boost/version.hpp"

Install-ZipDependency `
    -Name "GTSAM 4.2.0 source" `
    -Url "https://codeload.github.com/borglab/gtsam/zip/refs/tags/4.2.0" `
    -ArchiveName "gtsam-4.2.0.zip" `
    -InstallDirName "gtsam-4.2.0" `
    -RequiredFile "CMakeLists.txt"

$boostRoot = Join-Path $thirdPartyRoot "boost-1.82.0"
$boostLibraryDir = Join-Path $boostRoot "stage\lib"
$boostSerializationLibrary = Get-ChildItem -LiteralPath $boostLibraryDir -Filter "*serialization*.lib" -ErrorAction SilentlyContinue | Select-Object -First 1
if ($null -eq $boostSerializationLibrary -or $Force) {
    Write-Host "Building Boost libraries..."
    Push-Location $boostRoot
    try {
        & cmd.exe /d /c "call `"$vcvarsPath`" >nul && call bootstrap.bat vc143"
        if ($LASTEXITCODE -ne 0) {
            throw "Boost bootstrap failed with exit code $LASTEXITCODE"
        }
        $boostClPath = $clPath.Replace('\', '/')
        $boostVcvarsPath = $vcvarsPath.Replace('\', '/')
        $boostProjectConfig = @"
import option ;
using msvc : 14.3 : "$boostClPath" : <setup>"$boostVcvarsPath" ;
option.set keep-going : false ;
"@
        Set-Content -LiteralPath (Join-Path $boostRoot "project-config.jam") `
            -Value $boostProjectConfig `
            -Encoding Ascii
        & .\b2.exe `
            --with-serialization `
            --with-system `
            --with-filesystem `
            --with-thread `
            --with-program_options `
            --with-date_time `
            --with-timer `
            --with-chrono `
            --with-regex `
            variant=debug,release `
            link=static `
            runtime-link=shared `
            address-model=64 `
            threading=multi `
            toolset=msvc-14.3 `
            stage `
            -j $env:NUMBER_OF_PROCESSORS
        if ($LASTEXITCODE -ne 0) {
            throw "Boost build failed with exit code $LASTEXITCODE"
        }
    }
    finally {
        Pop-Location
    }
}

$gtsamSource = Join-Path $thirdPartyRoot "gtsam-4.2.0"
$gtsamBuild = Join-Path $thirdPartyRoot ".build\gtsam-4.2.0"
$gtsamInstall = Join-Path $thirdPartyRoot "gtsam-4.2.0-install"
$gtsamConfig = Join-Path $gtsamInstall "CMake\GTSAMConfig.cmake"
if (-not (Test-Path -LiteralPath $gtsamConfig) -or $Force) {
    if ($Force) {
        if (Test-Path -LiteralPath $gtsamBuild) {
            Remove-Item -LiteralPath $gtsamBuild -Recurse -Force
        }
        if (Test-Path -LiteralPath $gtsamInstall) {
            Remove-Item -LiteralPath $gtsamInstall -Recurse -Force
        }
    }
    $vsMajor = ([version]$vsVersion).Major
    $defaultGenerator = switch ($vsMajor) {
        18 { "Visual Studio 18 2026" }
        17 { "Visual Studio 17 2022" }
        16 { "Visual Studio 16 2019" }
        15 { "Visual Studio 15 2017" }
        default { throw "Unsupported Visual Studio version: $vsMajor" }
    }
    $generator = if ($env:CMAKE_GENERATOR) { $env:CMAKE_GENERATOR } else { $defaultGenerator }
    $platform = if ($env:CMAKE_PLATFORM) { $env:CMAKE_PLATFORM } else { "x64" }
    $boostConfig = Join-Path $boostLibraryDir "cmake\Boost-1.82.0"
    Write-Host "Configuring GTSAM 4.2.0..."
    & cmake `
        -S $gtsamSource `
        -B $gtsamBuild `
        -G $generator `
        -A $platform `
        "-DCMAKE_INSTALL_PREFIX=$gtsamInstall" `
        "-DEigen3_DIR=$eigenConfig" `
        "-DBOOST_ROOT=$boostRoot" `
        "-DBOOST_LIBRARYDIR=$boostLibraryDir" `
        "-DBoost_DIR=$boostConfig" `
        -DBoost_COMPILER=-vc143 `
        "-DCMAKE_POLICY_VERSION_MINIMUM=3.5" `
        -DCMAKE_POLICY_DEFAULT_CMP0167=NEW `
        -DCMAKE_POLICY_DEFAULT_CMP0057=NEW `
        -DBoost_NO_SYSTEM_PATHS=ON `
        -DGTSAM_USE_SYSTEM_EIGEN=ON `
        -DGTSAM_BUILD_TESTS=OFF `
        -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF `
        -DGTSAM_BUILD_UNSTABLE=OFF `
        -DGTSAM_BUILD_PYTHON=OFF `
        -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF `
        -DBUILD_SHARED_LIBS=OFF `
        -DGTSAM_WITH_TBB=OFF `
        "-DGTSAM_COMPILE_OPTIONS_PRIVATE_COMMON=/W3;/GR;/EHsc;/Zm300"
    if ($LASTEXITCODE -ne 0) {
        throw "GTSAM configure failed with exit code $LASTEXITCODE"
    }

    foreach ($configuration in @("Release", "Debug")) {
        Write-Host "Building and installing GTSAM $configuration..."
        & cmake --build $gtsamBuild --target install --config $configuration --parallel 1
        if ($LASTEXITCODE -ne 0) {
            throw "GTSAM $configuration build failed with exit code $LASTEXITCODE"
        }
    }
}

Write-Host "Third-party dependencies are ready."
