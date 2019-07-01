@echo off

set DEPLOY_PATH="A:\EmbededMontiArc\basic-simulator\install"
set DEPLOY_PATH2="A:\EmbededMontiArc\RMIModelServer\src\main\resources"

call build_release.bat
copy build\Release\HardwareEmulator.dll %DEPLOY_PATH%
copy build\Release\HardwareEmulator.dll %DEPLOY_PATH2%
