@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     build_local.bat 'folder'
    echo Description:
    echo     Builds the project for the local architecture.
    exit /B 0
)

pushd %~dp0
CALL get_config.bat config

set SHARED_CPP_PATH=../../externals/shared_cpp

cd %1

cmake -S . -B local_build
cmake --build local_build

popd