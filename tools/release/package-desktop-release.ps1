param(
    [string]$Version = "0.1.0",
    [string]$BuildDir = "",
    [string]$DistRoot = "",
    [string]$QtRoot = "D:/software/Qt/5.15.2/mingw81_64",
    [string]$MingwRoot = "D:/software/Qt/Tools/mingw810_64",
    [string]$CmakeExe = "D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/CMake/bin/cmake.exe",
    [string]$NinjaDir = "D:/software/Microsoft Visual Studio/2022/Community/Common7/IDE/CommonExtensions/Microsoft/CMake/Ninja",
    [int]$Parallel = 2
)

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$RepoRoot = Split-Path -Parent (Split-Path -Parent $ScriptDir)

if ([string]::IsNullOrWhiteSpace($BuildDir)) {
    $BuildDir = Join-Path $RepoRoot "build-release"
}

if ([string]::IsNullOrWhiteSpace($DistRoot)) {
    $DistRoot = Join-Path $RepoRoot "dist"
}

$PackageDir = Join-Path $DistRoot ("RoboSDP-{0}-win64" -f $Version)
$DesktopExe = Join-Path $BuildDir "apps/desktop-qt/RoboSDPDesktop.exe"
$PackageExe = Join-Path $PackageDir "RoboSDPDesktop.exe"
$WindeployqtExe = Join-Path $QtRoot "bin/windeployqt.exe"

if (-not (Test-Path $CmakeExe)) {
    throw "Cannot find cmake.exe: $CmakeExe"
}

if (-not (Test-Path $WindeployqtExe)) {
    throw "Cannot find windeployqt.exe: $WindeployqtExe"
}

# Build only the desktop executable for release packaging.
$env:PATH = "$QtRoot/bin;$MingwRoot/bin;$NinjaDir;" + $env:PATH

Write-Host "=== RoboSDP Release Package ==="
Write-Host "RepoRoot   : $RepoRoot"
Write-Host "BuildDir   : $BuildDir"
Write-Host "PackageDir : $PackageDir"
Write-Host "Version    : $Version"

& $CmakeExe `
    -S $RepoRoot `
    -B $BuildDir `
    -G Ninja `
    -DCMAKE_BUILD_TYPE=Release `
    "-DCMAKE_PREFIX_PATH=$QtRoot" `
    "-DCMAKE_C_COMPILER=$MingwRoot/bin/gcc.exe" `
    "-DCMAKE_CXX_COMPILER=$MingwRoot/bin/g++.exe" `
    -DROBOSDP_INCLUDE_TESTS_IN_ALL=OFF `
    -DROBOSDP_INCLUDE_VTK_SMOKE_IN_ALL=OFF

& $CmakeExe --build $BuildDir --target RoboSDPDesktop --parallel $Parallel

if (Test-Path $PackageDir) {
    Remove-Item -LiteralPath $PackageDir -Recurse -Force
}

New-Item -ItemType Directory -Force -Path $PackageDir | Out-Null

Copy-Item -LiteralPath $DesktopExe -Destination $PackageExe -Force
Copy-Item -LiteralPath (Join-Path $RepoRoot "resources") -Destination (Join-Path $PackageDir "resources") -Recurse -Force

# Deploy Qt runtime, platform plugins, and MinGW runtime libraries.
& $WindeployqtExe --release --compiler-runtime $PackageExe

function Copy-FileIfExists {
    param(
        [string]$SourceFile,
        [string]$TargetDir
    )

    if (Test-Path $SourceFile) {
        Copy-Item -LiteralPath $SourceFile -Destination (Join-Path $TargetDir (Split-Path -Leaf $SourceFile)) -Force
    }
}

function Copy-DirectoryIfExists {
    param(
        [string]$SourceDir,
        [string]$TargetDir
    )

    if (Test-Path $SourceDir) {
        Copy-Item -LiteralPath $SourceDir -Destination $TargetDir -Recurse -Force
    }
}

function Copy-DllsFromDir {
    param(
        [string]$SourceDir,
        [string]$TargetDir
    )

    if (-not (Test-Path $SourceDir)) {
        return
    }

    Get-ChildItem -Path $SourceDir -Filter *.dll -File | ForEach-Object {
        Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $TargetDir $_.Name) -Force
    }
}

# windeployqt may miss plugins in some local MinGW shells, so keep a deterministic fallback.
$QtBinDir = Join-Path $QtRoot "bin"
$QtPluginDir = Join-Path $QtRoot "plugins"

@(
    "Qt5Core.dll",
    "Qt5Gui.dll",
    "Qt5Widgets.dll",
    "Qt5OpenGL.dll"
) | ForEach-Object {
    Copy-FileIfExists -SourceFile (Join-Path $QtBinDir $_) -TargetDir $PackageDir
}

@(
    "libgcc_s_seh-1.dll",
    "libstdc++-6.dll",
    "libwinpthread-1.dll"
) | ForEach-Object {
    Copy-FileIfExists -SourceFile (Join-Path (Join-Path $MingwRoot "bin") $_) -TargetDir $PackageDir
}

@(
    "platforms",
    "styles",
    "imageformats",
    "iconengines"
) | ForEach-Object {
    Copy-DirectoryIfExists -SourceDir (Join-Path $QtPluginDir $_) -TargetDir (Join-Path $PackageDir $_)
}

# VTK is not handled by windeployqt, so copy known local VTK runtime folders.
$KnownVtkBinDirs = @(
    (Join-Path $RepoRoot "../_thirdparty-build/vtk-qt5-mingw81/install-iogeometry/bin"),
    (Join-Path $RepoRoot "../_thirdparty-build/vtk-qt5-mingw81/install-ready/bin")
)

foreach ($VtkBinDir in $KnownVtkBinDirs) {
    Copy-DllsFromDir -SourceDir $VtkBinDir -TargetDir $PackageDir
}

# Extra third-party DLL folders can be appended through a semicolon-separated environment variable.
if (-not [string]::IsNullOrWhiteSpace($env:ROBOSDP_EXTRA_DLL_DIRS)) {
    $env:ROBOSDP_EXTRA_DLL_DIRS.Split(';') | Where-Object { -not [string]::IsNullOrWhiteSpace($_) } | ForEach-Object {
        Copy-DllsFromDir -SourceDir $_ -TargetDir $PackageDir
    }
}

Write-Host "=== Release Package Finished ==="
Write-Host "Executable: $PackageExe"
