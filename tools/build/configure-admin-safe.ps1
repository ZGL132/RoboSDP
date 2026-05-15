param(
    [string]$BuildDir = "",
    [string]$QtRoot = "D:/software/Qt/5.15.2/mingw81_64",
    [string]$MingwRoot = "D:/software/Qt/Tools/mingw810_64",
    [string]$VtkCMakeDir = "D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/lib/cmake/vtk-9.2"
)

$ErrorActionPreference = "Stop"

# 通过脚本目录反推仓库根目录
$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    # 固定使用已经验通的管理员构建目录
    $BuildDir = Join-Path $RepoRoot "bdv3-admin"
}

# 当前机器普通权限下会触发 CMake 临时文件锁问题
# 这里明确要求管理员权限运行
$CurrentIdentity = [Security.Principal.WindowsIdentity]::GetCurrent()
$Principal = New-Object Security.Principal.WindowsPrincipal($CurrentIdentity)
if (-not $Principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) {
    throw "请以管理员权限运行 configure-admin-safe.ps1"
}

$NinjaExe = "D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja/ninja.exe"
$CmakeExe = "D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe"
$GccExe = "$MingwRoot/bin/gcc.exe"
$GxxExe = "$MingwRoot/bin/g++.exe"
$VtkBinDir = "D:/10_Source_Repos/21_robot/_thirdparty-build/vtk-qt5-mingw81/install-ready/bin"

if (-not (Test-Path $CmakeExe)) { throw "未找到 cmake.exe $CmakeExe" }
if (-not (Test-Path $NinjaExe)) { throw "未找到 ninja.exe $NinjaExe" }
if (-not (Test-Path $GccExe)) { throw "未找到 gcc.exe $GccExe" }
if (-not (Test-Path $GxxExe)) { throw "未找到 g++.exe $GxxExe" }
if (-not (Test-Path $VtkCMakeDir)) { throw "未找到 VTK CMake 包目录 $VtkCMakeDir" }

# 配置阶段补齐 Qt MinGW VTK 和 Ninja 的可执行路径
$env:PATH = "$QtRoot/bin;$MingwRoot/bin;$VtkBinDir;D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja;" + $env:PATH

Write-Host "=== RoboSDP 管理员配置 ==="
Write-Host "RepoRoot  : $RepoRoot"
Write-Host "BuildDir  : $BuildDir"
Write-Host "QtRoot    : $QtRoot"
Write-Host "MingwRoot : $MingwRoot"
Write-Host "VTK_DIR   : $VtkCMakeDir"

# 继续沿用已经验通的 configure 参数
& $CmakeExe `
    -S $RepoRoot `
    -B $BuildDir `
    -G Ninja `
    -DCMAKE_MAKE_PROGRAM=$NinjaExe `
    -DCMAKE_PREFIX_PATH=$QtRoot `
    -DCMAKE_C_COMPILER=$GccExe `
    -DCMAKE_CXX_COMPILER=$GxxExe `
    -DCMAKE_C_COMPILER_WORKS=1 `
    -DCMAKE_CXX_COMPILER_WORKS=1 `
    -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY `
    -DROBOSDP_VTK_DIR=$VtkCMakeDir
