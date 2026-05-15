@echo off
setlocal EnableExtensions EnableDelayedExpansion

set "ROBOSDP_REAL_COMPILER=%~1"
if "%ROBOSDP_REAL_COMPILER%"=="" (
    exit /b 1
)
shift

set "ROBOSDP_FILTERED_ARGS="
:collect_args
if "%~1"=="" goto invoke_compiler
if /I "%~1"=="/bigobj" (
    shift
    goto collect_args
)

set "ROBOSDP_FILTERED_ARGS=!ROBOSDP_FILTERED_ARGS! "%~1""
shift
goto collect_args

:invoke_compiler
call "%ROBOSDP_REAL_COMPILER%" %ROBOSDP_FILTERED_ARGS%
exit /b %errorlevel%
