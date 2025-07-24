@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     ema_build_local.bat 'folder' [Config]
    echo Description:
    echo     Compiles the CPP files for the EMA project specified in the 'folder'/ema_config file
    echo     Local compile = for this machine
    echo     Config: optional, specifies Release or Debug, defaults to Release
    exit /B 0
)

pushd %~dp0

call get_config.bat config
call get_config.bat %1/ema_config

set ARMADILLO_PATH=%~dp0%EXTERNALS_DIRECTORY%/armadillo
set CMAKE_ARGS=-DARMADILLO_PATH="%ARMADILLO_PATH%"

set BUILD_CONFIG=%2
IF ["%BUILD_CONFIG%"] == [""] (SET BUILD_CONFIG=Release)
set CMAKE_ARGS=%CMAKE_ARGS% -DCMAKE_BUILD_TYPE=%BUILD_CONFIG%

IF NOT [%GENERATOR%] == [] (
    set CMAKE_ARGS=%CMAKE_ARGS% -G"%GENERATOR%"
)

echo CMAKE_ARGS=%CMAKE_ARGS%

set BUILD_DIR=local_build

cd %1\target

cmake %CMAKE_ARGS% -S cpp -B %BUILD_DIR%
cmake --build %BUILD_DIR% --config %BUILD_CONFIG%
if NOT ["%DLL_DIR%"] == ["."] (
    cmake --install %BUILD_DIR% --config %BUILD_CONFIG%
)

popd
