@echo off
for %%I in (.) do set CurrDirName=%%~nxI
IF "%CurrDirName%"=="scripts" ( cd .. )
cd samples

echo **************************************
echo        Updating simple sample
echo **************************************
cd simple
call make.bat
copy "sample_simple.dll" "..\..\hardware_emulator\bin\sample_simple.dll"
cd ..

echo **************************************
echo      Updating funccalling sample
echo **************************************
cd funccalling
call make.bat
copy "sample_functioncalling.dll" "..\..\hardware_emulator\bin\sample_functioncalling.dll"
cd ..

echo **************************************
echo       Updating syscall sample
echo **************************************
cd syscall_dll
call make.bat
copy "sample_syscall.dll" "..\..\hardware_emulator\bin\sample_syscall.dll"
cd ..