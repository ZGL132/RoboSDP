param(
    [string]$BuildDir = "",
    [string]$QtRoot = "D:/software/Qt/5.15.2/mingw81_64",
    [string]$MingwRoot = "D:/software/Qt/Tools/mingw810_64",
    [string]$VtkBinDir = "D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/bin",
    [int]$WaitSeconds = 5
)

$ErrorActionPreference = "Stop"

# 固定默认构建目录，避免误测到别的构建输出。
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)
if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot "bdv3-admin"
}

# 启动自检沿用管理员路径，避免当前机器普通权限下的环境差异干扰判断。
$CurrentIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
$Principal = New-Object Security.Principal.WindowsPrincipal($CurrentIdentity)
if (-not $Principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "请以管理员权限运行 test-desktop-admin-safe.ps1"
}

$ExePath = Join-Path $BuildDir "apps/desktop-qt/RoboSDPDesktop.exe"
if (-not (Test-Path $ExePath)) {
    throw "未找到 RoboSDPDesktop.exe 请先运行 configure-admin-safe.ps1 和 build-desktop-admin-safe.ps1"
}

# 补齐桌面端运行所需的 Qt MinGW 和 VTK DLL 路径。
$env:PATH = "$QtRoot/bin;$MingwRoot/bin;$VtkBinDir;" + $env:PATH

Write-Host "=== RoboSDP 桌面端 5 秒启动自检 ==="
Write-Host "ExePath     : $ExePath"
Write-Host "WaitSeconds : $WaitSeconds"

$Process = Start-Process -FilePath $ExePath -PassThru
Start-Sleep -Seconds $WaitSeconds

if ($Process.HasExited) {
    Write-Output ("EXITED:" + $Process.ExitCode)
}
else {
    Write-Output "RUNNING_OK"
    Stop-Process -Id $Process.Id -Force
}
