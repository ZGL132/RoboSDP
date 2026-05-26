@echo off
md .\source
set copy_target_path=.\source\
for /r %%i in (*.h) do (copy /y "%%i" %copy_target_path%)
pause