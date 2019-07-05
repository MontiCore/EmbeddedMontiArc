@echo off
pushd %~dp0
echo.
echo **************************************
echo  Building Hardware Emulator (Release)
echo **************************************
call ..\scripts\build.bat
msbuild -verbosity:quiet build\hardware-emulator.sln /m /p:Configuration=Release /p:Platform=x64
