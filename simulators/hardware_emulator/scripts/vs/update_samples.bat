@REM
@REM (c) https://github.com/MontiCore/monticore
@REM




@REM -------------------------------------------------------------------------------------------------------------
@REM This script re-compiles all the sample programs used in the hardware_emulator tests.
@REM -------------------------------------------------------------------------------------------------------------

@echo off
pushd %~dp0\..\..
cd samples

echo [SCRIPT] Updating simple sample...
cd simple
call make.bat
echo [SCRIPT] Copying simple sample...
move "sample_simple.dll" "..\..\hardware_emulator\bin\sample_simple.dll"
cd ..

echo [SCRIPT] Updating funccalling sample...
cd funccalling
call make.bat
echo [SCRIPT] Copying funccalling sample...
move "sample_functioncalling.dll" "..\..\hardware_emulator\bin\sample_functioncalling.dll"
cd ..

echo [SCRIPT] Updating syscall sample...
cd syscall_dll
call make.bat
echo [SCRIPT] Copying syscall sample...
move "sample_syscall.dll" "..\..\hardware_emulator\bin\sample_syscall.dll"
cd ..
