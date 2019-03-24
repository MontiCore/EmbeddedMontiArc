@echo off
for %%I in (.) do set CurrDirName=%%~nxI
IF "%CurrDirName%"=="scripts" ( cd .. )
cd pe-parse
rmdir /S /Q build
cd ..

cd unicorn
rmdir /S /Q build
cd ..

cd zydis
rmdir /S /Q build
cd ..


cd hardware_emulator
rmdir /S /Q build
cd ..