@echo off
setlocal EnableExtensions DisableDelayedExpansion
chcp 65001 >nul

REM ============================================================
REM LivoxViewerQT - Windows portable + installer package script
REM Purpose: build Release, generate portable directory, ZIP package,
REM          and compile an Inno Setup EXE installer.
REM Run this file from the scripts directory or any working directory.
REM Usage:
REM   package_windows_release.bat
REM   package_windows_release.bat incremental
REM ============================================================

set "APP_NAME=LivoxViewerQT"
set "APP_EXE=LivoxViewerQT.exe"
set "APP_PUBLISHER=@FelixCooper1026"
set "APP_URL=https://github.com/FelixCooper1026"
set "APP_DESCRIPTION=Livox LiDAR Qt Viewer Application"

REM ---- Use environment variables or discover installed tools ----
if not defined QT_DIR if defined QT_ROOT_DIR set "QT_DIR=%QT_ROOT_DIR%"
if not defined QT_DIR for /f "delims=" %%i in ('where qmake.exe 2^>nul') do if not defined QT_DIR for %%j in ("%%~dpi..") do set "QT_DIR=%%~fj"
if not defined VS_VCVARS if exist "%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" for /f "usebackq delims=" %%i in (`"%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "VS_VCVARS=%%i\VC\Auxiliary\Build\vcvars64.bat"
if not defined ISCC_EXE for /f "delims=" %%i in ('where ISCC.exe 2^>nul') do if not defined ISCC_EXE set "ISCC_EXE=%%i"
if not defined ISCC_EXE if exist "%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe" set "ISCC_EXE=%ProgramFiles(x86)%\Inno Setup 6\ISCC.exe"
if not defined ISCC_EXE if exist "%ProgramFiles%\Inno Setup 6\ISCC.exe" set "ISCC_EXE=%ProgramFiles%\Inno Setup 6\ISCC.exe"
if not defined CMAKE_GENERATOR set "CMAKE_GENERATOR=Visual Studio 17 2022"
if not defined CMAKE_PLATFORM set "CMAKE_PLATFORM=x64"
if not defined PACKAGE_BUILD_JOBS set "PACKAGE_BUILD_JOBS=2"

REM ---- Project paths ----
set "SCRIPT_DIR=%~dp0"
if "%SCRIPT_DIR:~-1%"=="\" set "SCRIPT_DIR=%SCRIPT_DIR:~0,-1%"
set "ROOT_DIR=%SCRIPT_DIR%\.."

set "BUILD_TYPE=Release"
set "BUILD_DIR=%ROOT_DIR%\build\cmd-msvc-package"
set "DIST_DIR=%ROOT_DIR%\dist\windows"
set "PORTABLE_DIR=%DIST_DIR%\%APP_NAME%_Portable"
set "INSTALLER_DIR=%DIST_DIR%\installer"
set "ISS_FILE=%INSTALLER_DIR%\%APP_NAME%_Setup.generated.iss"

set "CLEAN_BUILD=1"
:parse_args
if "%~1"=="" goto args_done
if /I "%~1"=="clean" set "CLEAN_BUILD=1"
if /I "%~1"=="incremental" set "CLEAN_BUILD=0"
shift
goto parse_args
:args_done

echo.
echo ==== LivoxViewerQT Windows Release Package ====
echo ROOT_DIR:        %ROOT_DIR%
echo BUILD_DIR:       %BUILD_DIR%
echo DIST_DIR:        %DIST_DIR%
echo PORTABLE_DIR:    %PORTABLE_DIR%
echo INSTALLER_DIR:   %INSTALLER_DIR%
echo QT_DIR:          %QT_DIR%
echo VS_VCVARS:       %VS_VCVARS%
echo ISCC_EXE:        %ISCC_EXE%
echo CMAKE_GENERATOR: %CMAKE_GENERATOR%
echo BUILD_JOBS:      %PACKAGE_BUILD_JOBS%
echo.

REM ============================================================
REM Checks
REM ============================================================

if not exist "%ROOT_DIR%\CMakeLists.txt" (
    echo ERROR: CMakeLists.txt not found. Put this script in the project root.
    if not defined CI pause
    exit /b 1
)

where cmake >nul 2>nul
if errorlevel 1 (
    echo ERROR: cmake not found in PATH.
    if not defined CI pause
    exit /b 1
)

where powershell >nul 2>nul
if errorlevel 1 (
    echo ERROR: powershell not found in PATH.
    if not defined CI pause
    exit /b 1
)

if not exist "%QT_DIR%\bin\qmake.exe" (
    echo ERROR: Qt not found. Set QT_DIR or QT_ROOT_DIR to the Qt installation prefix.
    if not defined CI pause
    exit /b 1
)

if not exist "%QT_DIR%\bin\windeployqt.exe" (
    echo ERROR: windeployqt not found: %QT_DIR%\bin\windeployqt.exe
    if not defined CI pause
    exit /b 1
)

if not exist "%ROOT_DIR%\resources\app_icon.ico" (
    echo ERROR: resources\app_icon.ico not found.
    if not defined CI pause
    exit /b 1
)

if not exist "%VS_VCVARS%" (
    echo ERROR: vcvars64.bat not found. Set VS_VCVARS or install Visual Studio with C++ tools.
    if not defined CI pause
    exit /b 1
)

if not exist "%ISCC_EXE%" (
    echo ERROR: Inno Setup 6 not found. Set ISCC_EXE or install Inno Setup 6.
    if not defined CI pause
    exit /b 1
)

REM ============================================================
REM Read version from CMakeLists.txt
REM ============================================================

for /f "usebackq delims=" %%v in (`powershell -NoProfile -ExecutionPolicy Bypass -Command "$t=Get-Content -Raw -LiteralPath '%ROOT_DIR%\CMakeLists.txt'; $m=[regex]::Match($t,'project\s*\(\s*LivoxViewerQT[\s\S]*?VERSION\s+([0-9]+(?:\.[0-9]+){0,3})'); if(!$m.Success){exit 1}; $m.Groups[1].Value"`) do set "APP_VERSION=%%v"

if "%APP_VERSION%"=="" (
    echo ERROR: Failed to read project VERSION from CMakeLists.txt.
    if not defined CI pause
    exit /b 1
)

set "PORTABLE_ZIP=%DIST_DIR%\%APP_NAME%-%APP_VERSION%-win64-portable.zip"
set "INSTALLER_EXE=%INSTALLER_DIR%\%APP_NAME%_Setup_v%APP_VERSION%_x64.exe"

echo App version: %APP_VERSION%

REM ============================================================
REM Setup compiler and Qt environment
REM ============================================================

echo.
echo ==== Setup MSVC environment ====
call "%VS_VCVARS%"
if errorlevel 1 (
    echo ERROR: Failed to initialize MSVC environment.
    if not defined CI pause
    exit /b 1
)

set "PATH=%QT_DIR%\bin;%PATH%"
set "MSBUILDDISABLENODEREUSE=1"

echo.
echo Qt check:
"%QT_DIR%\bin\qmake.exe" -v

REM ============================================================
REM Stop running process and force icon resource rebuild
REM ============================================================

echo.
echo ==== Stop running program ====
taskkill /f /im "%APP_EXE%" 2>nul

if exist "%ROOT_DIR%\resources\app_icon.rc" (
    echo Touch app_icon.rc to force icon resource rebuild...
    powershell -NoProfile -ExecutionPolicy Bypass -Command "(Get-Item -LiteralPath '%ROOT_DIR%\resources\app_icon.rc').LastWriteTime = Get-Date"
)

REM ============================================================
REM Clean if requested
REM ============================================================

if "%CLEAN_BUILD%"=="1" (
    echo.
    echo ==== Clean build directory ====
    if exist "%BUILD_DIR%" rmdir /s /q "%BUILD_DIR%"
)

REM ============================================================
REM Configure and build Release
REM ============================================================

echo.
echo ==== Configure CMake ====
cmake -S "%ROOT_DIR%" -B "%BUILD_DIR%" ^
  -G "%CMAKE_GENERATOR%" -A "%CMAKE_PLATFORM%" ^
  -DCMAKE_PREFIX_PATH="%QT_DIR%"

if errorlevel 1 (
    echo ERROR: CMake configure failed.
    if not defined CI pause
    exit /b 1
)

echo.
echo ==== Build Release ====
cmake --build "%BUILD_DIR%" --target "%APP_NAME%" --config "%BUILD_TYPE%" --parallel %PACKAGE_BUILD_JOBS% -- /nodeReuse:false /p:CL_MPCount=%PACKAGE_BUILD_JOBS%
if errorlevel 1 (
    echo ERROR: Build failed.
    if not defined CI pause
    exit /b 1
)

set "BUILD_EXE="
if exist "%BUILD_DIR%\%BUILD_TYPE%\%APP_EXE%" set "BUILD_EXE=%BUILD_DIR%\%BUILD_TYPE%\%APP_EXE%"
if exist "%BUILD_DIR%\%APP_EXE%" set "BUILD_EXE=%BUILD_DIR%\%APP_EXE%"

if "%BUILD_EXE%"=="" (
    echo ERROR: Cannot find built executable.
    echo Checked:
    echo   %BUILD_DIR%\%BUILD_TYPE%\%APP_EXE%
    echo   %BUILD_DIR%\%APP_EXE%
    if not defined CI pause
    exit /b 1
)

for %%I in ("%BUILD_EXE%") do set "BUILD_EXE_DIR=%%~dpI"
if "%BUILD_EXE_DIR:~-1%"=="\" set "BUILD_EXE_DIR=%BUILD_EXE_DIR:~0,-1%"

set "OPENCV_RUNTIME_NAME="
for /f "tokens=*" %%F in ('dumpbin /nologo /dependents "%BUILD_EXE%" ^| findstr /i "opencv_world"') do set "OPENCV_RUNTIME_NAME=%%F"
if not defined OPENCV_RUNTIME_NAME (
    echo ERROR: OpenCV runtime dependency was not found in %BUILD_EXE%.
    if not defined CI pause
    exit /b 1
)
if not exist "%BUILD_EXE_DIR%\%OPENCV_RUNTIME_NAME%" (
    echo ERROR: OpenCV runtime DLL missing: %BUILD_EXE_DIR%\%OPENCV_RUNTIME_NAME%
    if not defined CI pause
    exit /b 1
)

echo Built exe: %BUILD_EXE%
echo OpenCV runtime: %OPENCV_RUNTIME_NAME%

REM ============================================================
REM Rebuild portable directory
REM ============================================================

echo.
echo ==== Rebuild portable directory ====

if exist "%PORTABLE_DIR%" rmdir /s /q "%PORTABLE_DIR%"
mkdir "%PORTABLE_DIR%"
if not exist "%INSTALLER_DIR%" mkdir "%INSTALLER_DIR%"

echo Copy executable...
copy /y "%BUILD_EXE%" "%PORTABLE_DIR%\" >nul
if errorlevel 1 (
    echo ERROR: Failed to copy executable.
    if not defined CI pause
    exit /b 1
)

echo Copy OpenCV runtime...
copy /y "%BUILD_EXE_DIR%\%OPENCV_RUNTIME_NAME%" "%PORTABLE_DIR%\" >nul
if errorlevel 1 (
    echo ERROR: Failed to copy %OPENCV_RUNTIME_NAME%.
    if not defined CI pause
    exit /b 1
)

echo Copy application icon...
copy /y "%ROOT_DIR%\resources\app_icon.ico" "%PORTABLE_DIR%\app_icon.ico" >nul

echo Copy configuration files...
if exist "%BUILD_EXE_DIR%\config.json" copy /y "%BUILD_EXE_DIR%\config.json" "%PORTABLE_DIR%\" >nul
if exist "%BUILD_EXE_DIR%\mid360_config.json" copy /y "%BUILD_EXE_DIR%\mid360_config.json" "%PORTABLE_DIR%\" >nul
if not exist "%PORTABLE_DIR%\config.json" if exist "%ROOT_DIR%\config.json" copy /y "%ROOT_DIR%\config.json" "%PORTABLE_DIR%\" >nul
if not exist "%PORTABLE_DIR%\mid360_config.json" if exist "%ROOT_DIR%\mid360_config.json" copy /y "%ROOT_DIR%\mid360_config.json" "%PORTABLE_DIR%\" >nul

echo Copy Livox SDK files...
if exist "%BUILD_EXE_DIR%\livox_sdk_qt" (
    xcopy "%BUILD_EXE_DIR%\livox_sdk_qt" "%PORTABLE_DIR%\livox_sdk_qt" /E /I /Y >nul
) else if exist "%ROOT_DIR%\livox_sdk_qt" (
    xcopy "%ROOT_DIR%\livox_sdk_qt" "%PORTABLE_DIR%\livox_sdk_qt" /E /I /Y >nul
) else (
    echo WARNING: livox_sdk_qt directory not found.
)

REM ============================================================
REM Deploy Qt runtime
REM ============================================================

echo.
echo ==== Run windeployqt ====
"%QT_DIR%\bin\windeployqt.exe" ^
  --release ^
  --compiler-runtime ^
  --no-translations ^
  --no-system-d3d-compiler ^
  --no-opengl-sw ^
  "%PORTABLE_DIR%\%APP_EXE%"

if errorlevel 1 (
    echo ERROR: windeployqt failed.
    if not defined CI pause
    exit /b 1
)

REM ============================================================
REM Generate portable README and launcher
REM ============================================================

echo.
echo ==== Generate portable files ====

> "%PORTABLE_DIR%\README.txt" echo %APP_NAME% Portable Version
>>"%PORTABLE_DIR%\README.txt" echo.
>>"%PORTABLE_DIR%\README.txt" echo Version: %APP_VERSION%
>>"%PORTABLE_DIR%\README.txt" echo Qt: 6.8.3 MSVC x64
>>"%PORTABLE_DIR%\README.txt" echo.
>>"%PORTABLE_DIR%\README.txt" echo Usage:
>>"%PORTABLE_DIR%\README.txt" echo 1. Run %APP_EXE% directly.
>>"%PORTABLE_DIR%\README.txt" echo 2. Ensure the Livox device and host are on the correct network segment.
>>"%PORTABLE_DIR%\README.txt" echo 3. If device discovery fails, check firewall, Npcap, and network adapter settings.

> "%PORTABLE_DIR%\StartLivoxViewerQT.bat" echo @echo off
>>"%PORTABLE_DIR%\StartLivoxViewerQT.bat" echo cd /d "%%~dp0"
>>"%PORTABLE_DIR%\StartLivoxViewerQT.bat" echo start "" "%APP_EXE%"

REM ============================================================
REM Validate portable directory
REM ============================================================

echo.
echo ==== Validate portable package ====

if not exist "%PORTABLE_DIR%\%APP_EXE%" (
    echo ERROR: Main executable missing.
    if not defined CI pause
    exit /b 1
)

if not exist "%PORTABLE_DIR%\app_icon.ico" (
    echo ERROR: app_icon.ico missing.
    if not defined CI pause
    exit /b 1
)

if not exist "%PORTABLE_DIR%\platforms\qwindows.dll" (
    echo ERROR: Qt platform plugin missing: platforms\qwindows.dll
    if not defined CI pause
    exit /b 1
)

if not exist "%PORTABLE_DIR%\Qt6Core.dll" (
    echo ERROR: Qt6Core.dll missing.
    if not defined CI pause
    exit /b 1
)

if not exist "%PORTABLE_DIR%\%OPENCV_RUNTIME_NAME%" (
    echo ERROR: OpenCV runtime missing: %OPENCV_RUNTIME_NAME%
    if not defined CI pause
    exit /b 1
)

echo Portable package OK.

REM ============================================================
REM Create portable ZIP
REM ============================================================

echo.
echo ==== Create portable ZIP ====
if exist "%PORTABLE_ZIP%" del /f /q "%PORTABLE_ZIP%"
powershell -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; $source='%PORTABLE_DIR%\*'; $dest='%PORTABLE_ZIP%'; for($i=1; $i -le 5; $i++){ try { Compress-Archive -Path $source -DestinationPath $dest -Force; exit 0 } catch { if($i -eq 5){ Write-Error $_; exit 1 }; Start-Sleep -Seconds 2 } }"
if errorlevel 1 (
    echo ERROR: Failed to create portable ZIP.
    if not defined CI pause
    exit /b 1
)
if not exist "%PORTABLE_ZIP%" (
    echo ERROR: Portable ZIP was not created.
    if not defined CI pause
    exit /b 1
)

REM ============================================================
REM Generate Inno Setup script
REM ============================================================

echo.
echo ==== Generate Inno Setup script ====

if exist "%ISS_FILE%" del /f /q "%ISS_FILE%"

>>"%ISS_FILE%" echo ; Auto-generated by package_windows_release.bat. Do not edit manually.
>>"%ISS_FILE%" echo #define MyAppName "%APP_NAME%"
>>"%ISS_FILE%" echo #define MyAppVersion "%APP_VERSION%"
>>"%ISS_FILE%" echo #define MyAppPublisher "%APP_PUBLISHER%"
>>"%ISS_FILE%" echo #define MyAppURL "%APP_URL%"
>>"%ISS_FILE%" echo #define MyAppExeName "%APP_EXE%"
>>"%ISS_FILE%" echo #define MyAppDescription "%APP_DESCRIPTION%"
>>"%ISS_FILE%" echo #define MyPortableDir "%PORTABLE_DIR%"
>>"%ISS_FILE%" echo #define MyInstallerOutput "%INSTALLER_DIR%"
>>"%ISS_FILE%" echo #define MyAppIcon "%PORTABLE_DIR%\app_icon.ico"
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Setup]
>>"%ISS_FILE%" echo AppId={{8B5F3A2E-1C4D-4E5F-9A8B-2C3D4E5F6A7B}
>>"%ISS_FILE%" echo AppName={#MyAppName}
>>"%ISS_FILE%" echo AppVersion={#MyAppVersion}
>>"%ISS_FILE%" echo AppVerName={#MyAppName} {#MyAppVersion}
>>"%ISS_FILE%" echo AppPublisher={#MyAppPublisher}
>>"%ISS_FILE%" echo AppPublisherURL={#MyAppURL}
>>"%ISS_FILE%" echo AppSupportURL={#MyAppURL}
>>"%ISS_FILE%" echo AppUpdatesURL={#MyAppURL}
>>"%ISS_FILE%" echo DefaultDirName={autopf}\{#MyAppName}
>>"%ISS_FILE%" echo DefaultGroupName={#MyAppName}
>>"%ISS_FILE%" echo AllowNoIcons=yes
>>"%ISS_FILE%" echo DisableDirPage=no
>>"%ISS_FILE%" echo DisableProgramGroupPage=no
>>"%ISS_FILE%" echo OutputDir={#MyInstallerOutput}
>>"%ISS_FILE%" echo OutputBaseFilename={#MyAppName}_Setup_v{#MyAppVersion}_x64
>>"%ISS_FILE%" echo SetupIconFile={#MyAppIcon}
>>"%ISS_FILE%" echo UninstallDisplayIcon={app}\app_icon.ico
>>"%ISS_FILE%" echo Compression=lzma2
>>"%ISS_FILE%" echo SolidCompression=yes
>>"%ISS_FILE%" echo WizardStyle=modern
>>"%ISS_FILE%" echo PrivilegesRequired=admin
>>"%ISS_FILE%" echo ArchitecturesAllowed=x64
>>"%ISS_FILE%" echo ArchitecturesInstallIn64BitMode=x64
>>"%ISS_FILE%" echo CloseApplications=yes
>>"%ISS_FILE%" echo RestartApplications=no
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Languages]
>>"%ISS_FILE%" echo Name: "chinesesimp"; MessagesFile: "compiler:Languages\ChineseSimplified.isl"
>>"%ISS_FILE%" echo Name: "english"; MessagesFile: "compiler:Default.isl"
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Tasks]
>>"%ISS_FILE%" echo Name: "desktopicon"; Description: "{cm:CreateDesktopIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked
>>"%ISS_FILE%" echo Name: "quicklaunchicon"; Description: "{cm:CreateQuickLaunchIcon}"; GroupDescription: "{cm:AdditionalIcons}"; Flags: unchecked; OnlyBelowVersion: 6.1
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Files]
>>"%ISS_FILE%" echo Source: "{#MyPortableDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [InstallDelete]
>>"%ISS_FILE%" echo Type: files; Name: "{autodesktop}\{#MyAppName}.lnk"
>>"%ISS_FILE%" echo Type: files; Name: "{group}\{#MyAppName}.lnk"
>>"%ISS_FILE%" echo Type: files; Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#MyAppName}.lnk"
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Icons]
>>"%ISS_FILE%" echo Name: "{group}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\app_icon.ico"; IconIndex: 0; Comment: "{#MyAppDescription}"
>>"%ISS_FILE%" echo Name: "{group}\README"; Filename: "{app}\README.txt"
>>"%ISS_FILE%" echo Name: "{group}\{cm:UninstallProgram,{#MyAppName}}"; Filename: "{uninstallexe}"
>>"%ISS_FILE%" echo Name: "{autodesktop}\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\app_icon.ico"; IconIndex: 0; Tasks: desktopicon; Comment: "{#MyAppDescription}"
>>"%ISS_FILE%" echo Name: "{userappdata}\Microsoft\Internet Explorer\Quick Launch\{#MyAppName}"; Filename: "{app}\{#MyAppExeName}"; IconFilename: "{app}\app_icon.ico"; IconIndex: 0; Tasks: quicklaunchicon
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [Run]
>>"%ISS_FILE%" echo Filename: "{app}\{#MyAppExeName}"; Description: "Launch {#MyAppName}"; Flags: nowait postinstall skipifsilent
>>"%ISS_FILE%" echo.
>>"%ISS_FILE%" echo [UninstallDelete]
>>"%ISS_FILE%" echo Type: filesandordirs; Name: "{app}\config"
>>"%ISS_FILE%" echo Type: filesandordirs; Name: "{app}\logs"

echo Generated ISS: %ISS_FILE%

REM ============================================================
REM Build installer
REM ============================================================

echo.
echo ==== Build installer ====
"%ISCC_EXE%" "%ISS_FILE%"
if errorlevel 1 (
    echo ERROR: Inno Setup build failed.
    if not defined CI pause
    exit /b 1
)

REM ============================================================
REM Final output
REM ============================================================

echo.
echo ==== Done ====
echo Portable directory:
echo   %PORTABLE_DIR%
echo Portable ZIP:
echo   %PORTABLE_ZIP%
echo Installer EXE:
echo   %INSTALLER_EXE%
echo.
echo Recommended test:
echo   1. Run portable: %PORTABLE_DIR%\%APP_EXE%
echo   2. Install:      %INSTALLER_EXE%
echo.
if not defined CI pause
exit /b 0
