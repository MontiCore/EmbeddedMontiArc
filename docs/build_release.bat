@echo off
for %%I in (.) do set CurrDirName=%%~nxI
IF "%CurrDirName%"=="docs" ( cd .. )

echo.
echo **************************************
echo  Building Hardware Emulator (Release)
echo **************************************
cd hardware_emulator
call ..\docs\build.bat
msbuild -verbosity:quiet build\hardware-emulator.sln /m /p:Configuration=Release /p:Platform=x64
cd ..