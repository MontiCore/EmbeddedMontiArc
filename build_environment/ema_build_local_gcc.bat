@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     ema_build_local_gcc.bat 'folder'
    echo Description:
    echo     Compiles the CPP files for the EMA project specified in the 'folder'/ema_config file
    echo     Local compile = for this machine
    exit /B 0
)

pushd %~dp0

call get_config.bat config
call get_config.bat %1/ema_config

set ARMADILLO_PATH=%~dp0/%EXTERNALS_DIRECTORY%/armadillo

IF ["%BUILD_CONFIG%"] == [""] (SET BUILD_CONFIG=Release)

cd %1\target

cmake -DCMAKE_BUILD_TYPE=%BUILD_CONFIG% -G"MinGW Makefiles" -S cpp -B gcc_build

cmake --build gcc_build --config %BUILD_CONFIG%
if NOT ["%DLL_DIR%"] == ["."] (
    cmake --install gcc_build --config %BUILD_CONFIG%
)

popd