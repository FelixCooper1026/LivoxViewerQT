param(
    [switch]$Force
)

$ErrorActionPreference = "Stop"

$repoRoot = Split-Path -Parent $PSScriptRoot
$thirdPartyRoot = Join-Path $repoRoot "third-party"
$downloadRoot = Join-Path $thirdPartyRoot ".downloads"

New-Item -ItemType Directory -Force -Path $thirdPartyRoot | Out-Null
New-Item -ItemType Directory -Force -Path $downloadRoot | Out-Null

function Install-ZipDependency {
    param(
        [Parameter(Mandatory = $true)][string]$Name,
        [Parameter(Mandatory = $true)][string]$Url,
        [Parameter(Mandatory = $true)][string]$ArchiveName,
        [Parameter(Mandatory = $true)][string]$InstallDirName,
        [Parameter(Mandatory = $true)][string]$RequiredFile
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
    Expand-Archive -LiteralPath $archivePath -DestinationPath $extractRoot -Force

    $sourceRoot = Get-ChildItem -LiteralPath $extractRoot -Directory | Select-Object -First 1
    if ($null -eq $sourceRoot) {
        throw "Archive did not contain an extracted source directory: $archivePath"
    }

    New-Item -ItemType Directory -Force -Path $installDir | Out-Null
    Get-ChildItem -LiteralPath $sourceRoot.FullName -Force | Move-Item -Destination $installDir -Force

    if (-not (Test-Path -LiteralPath $requiredPath)) {
        throw "$Name installation failed; missing $requiredPath"
    }

    Write-Host "$Name installed: $installDir"
}

Install-ZipDependency `
    -Name "Eigen 3.4.0" `
    -Url "https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip" `
    -ArchiveName "eigen-3.4.0.zip" `
    -InstallDirName "eigen-3.4.0" `
    -RequiredFile "Eigen/Core"

Install-ZipDependency `
    -Name "Boost 1.82.0 headers" `
    -Url "https://archives.boost.io/release/1.82.0/source/boost_1_82_0.zip" `
    -ArchiveName "boost_1_82_0.zip" `
    -InstallDirName "boost-1.82.0" `
    -RequiredFile "boost/preprocessor/seq.hpp"

Write-Host "Third-party dependencies are ready."
