param(
    [string]$BuildDir = "",
    [string]$QtRoot = "D:/software/Qt/5.15.2/mingw81_64",
    [string]$MingwRoot = "D:/software/Qt/Tools/mingw810_64",
    [int]$Parallel = 16
)

$ErrorActionPreference = "Stop"

# 固定默认构建目录
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot "bdv3-admin"
}

# 构建阶段也要求管理员权限
$CurrentIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
$Principal = New-Object Security.Principal.WindowsPrincipal($CurrentIdentity)
if (-not $Principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "请以管理员权限运行 build-desktop-admin-safe.ps1"
}

$CmakeExe = "D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
$VtkBinDir = "D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/bin"

if (-not (Test-Path $CmakeExe)) { throw "未找到 cmake.exe $CmakeExe" }
if (-not (Test-Path $BuildDir)) { throw "构建目录不存在 请先运行 configure-admin-safe.ps1" }

# 构建时也补齐运行时和工具链路径
$env:PATH = "$QtRoot/bin;$MingwRoot/bin;$VtkBinDir;D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja;" + $env:PATH

Write-Host "=== RoboSDP 管理员构建 ==="
Write-Host "BuildDir : $BuildDir"
Write-Host "Target   : RoboSDPDesktop"
Write-Host "Parallel : $Parallel"

# 只编译桌面端目标
& $CmakeExe --build $BuildDir --target RoboSDPDesktop --parallel $Parallel
