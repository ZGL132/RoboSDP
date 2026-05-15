param(
    [string]$BuildDir = "",
    [string]$QtRoot = "D:/software/Qt/5.15.2/mingw81_64",
    [string]$MingwRoot = "D:/software/Qt/Tools/mingw810_64",
    [string]$VtkBinDir = "D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/bin"
)

$ErrorActionPreference = "Stop"

# 固定默认构建目录
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot "bdv3-admin"
}

$ExePath = Join-Path $BuildDir "apps/desktop-qt/RoboSDPDesktop.exe"
if (-not (Test-Path $ExePath)) {
    throw "未找到 RoboSDPDesktop.exe 请先运行 configure-admin-safe.ps1 和 build-desktop-admin-safe.ps1"
}

# 运行桌面端前补齐 Qt MinGW 和 VTK 的 DLL 路径
$env:PATH = "$QtRoot/bin;$MingwRoot/bin;$VtkBinDir;" + $env:PATH

Write-Host "=== RoboSDP 管理员运行 ==="
Write-Host "ExePath : $ExePath"
Write-Host "VTK Bin : $VtkBinDir"

# 这里只负责启动桌面程序
Start-Process -FilePath $ExePath
