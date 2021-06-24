@REM
@REM (c) https://github.com/MontiCore/monticore
@REM


set BASIC_SIM_FOLDER=..\basic-simulator

pushd %~dp0
call mvn clean install -s settings.xml -DskipTests
if %ERRORLEVEL% NEQ 0 (
    goto end
)

cd %BASIC_SIM_FOLDER%
if %ERRORLEVEL% NEQ 0 (
    echo Could not go to basic-simulator folder: %BASIC_SIM_FOLDER%
    goto end
)

call mvn clean install -s settings.xml
if %ERRORLEVEL% NEQ 0 (
    goto end
)

cd install

del hardware_emulator_lib.dll

start run.bat
popd
goto after_end

:end
popd
::exit \B %ERRORLEVEL%
echo Error building the chain

:after_end
