@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     ema_build_local.bat 'folder'
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

if ["%GENERATOR%"] == [""] (
    cmake -DCMAKE_BUILD_TYPE=%BUILD_CONFIG% -S cpp -B local_build
) else (
    cmake -DCMAKE_BUILD_TYPE=%BUILD_CONFIG% -G"%GENERATOR%" -S cpp -B local_build
)

cmake --build local_build --config %BUILD_CONFIG%
if NOT ["%DLL_DIR%"] == ["."] (
    cmake --install local_build --config %BUILD_CONFIG%
)

popd