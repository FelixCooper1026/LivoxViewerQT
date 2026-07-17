@echo off
setlocal

:: Usage: compile.bat [config] [build_dir] [target]
:: Environment overrides: LIVOX_BUILD_CONFIG, LIVOX_BUILD_DIR,
:: LIVOX_BUILD_TARGET, LIVOX_CMAKE_COMMAND, CMAKE_PREFIX_PATH.

for %%I in ("%~dp0..") do set "PROJECT_ROOT=%%~fI"

set "BUILD_CONFIG=%LIVOX_BUILD_CONFIG%"
if not defined BUILD_CONFIG set "BUILD_CONFIG=Release"
if not "%~1"=="" set "BUILD_CONFIG=%~1"

set "BUILD_DIR=%LIVOX_BUILD_DIR%"
if not defined BUILD_DIR set "BUILD_DIR=%PROJECT_ROOT%\build-msvc"
if not "%~2"=="" set "BUILD_DIR=%~2"

set "BUILD_TARGET=%LIVOX_BUILD_TARGET%"
if not defined BUILD_TARGET set "BUILD_TARGET=LivoxViewerQT"
if not "%~3"=="" set "BUILD_TARGET=%~3"

set "CMAKE_COMMAND=%LIVOX_CMAKE_COMMAND%"
if not defined CMAKE_COMMAND set "CMAKE_COMMAND=cmake"

echo Configuring %BUILD_CONFIG% in %BUILD_DIR%...
"%CMAKE_COMMAND%" -S "%PROJECT_ROOT%" -B "%BUILD_DIR%" -DCMAKE_BUILD_TYPE="%BUILD_CONFIG%"
if errorlevel 1 exit /b 1

echo Building %BUILD_TARGET%...
"%CMAKE_COMMAND%" --build "%BUILD_DIR%" --config "%BUILD_CONFIG%" --target "%BUILD_TARGET%" --parallel
if errorlevel 1 exit /b 1

echo Build completed successfully.
