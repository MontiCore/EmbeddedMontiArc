@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     build_local.bat 'folder' [Config]
    echo Description:
    echo     Builds the project for the local architecture.
    echo     Config: optional, specifies Release or Debug, defaults to Release
    exit /B 0
)

pushd %~dp0
CALL get_config.bat config

set SHARED_CPP_DIR=%~dp0%EXTERNALS_DIRECTORY%/shared_cpp
set CMAKE_ARGS=-DSHARED_CPP_DIR="%SHARED_CPP_DIR%"

set BUILD_CONFIG=%2
IF ["%BUILD_CONFIG%"] == [""] (SET BUILD_CONFIG=Release)
set CMAKE_ARGS=%CMAKE_ARGS% -DCMAKE_BUILD_TYPE=%BUILD_CONFIG%

echo CMAKE_ARGS=%CMAKE_ARGS%

set BUILD_DIR=local_build

cd %1

cmake %CMAKE_ARGS% -S . -B %BUILD_DIR%
cmake --build %BUILD_DIR% --config %BUILD_CONFIG%

popd
