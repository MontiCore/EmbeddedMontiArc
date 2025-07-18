@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     emam_generate.bat 'folder'
    echo Description:
    echo     Generates the CPP files for the EMAM project specified in the 'folder'/ema_config file
    echo Configuration:
    echo     The 'EMAM_TO_CPP_PROJECT' and 'EMAM_TO_CPP_VERSION' properties must be set inside the 'config' file
    exit /B 0
)

pushd %~dp0

call get_config.bat config
call get_config.bat %1/ema_config

set EMAM_TO_CPP_JAR=%EMAM_TO_CPP_PROJECT%\target\embedded-montiarc-math-generator-%EMAM_TO_CPP_VERSION%-jar-with-dependencies.jar

cd %1

set OUTPUT_DIR=target/cpp

@echo on

call java -jar %EMAM_TO_CPP_JAR% ^
  --models-dir=%MODEL_DIR% ^
  --root-model=%MODEL_NAME% ^
  --output-dir=%OUTPUT_DIR% ^
  --flag-use-armadillo-backend ^
  --armadillo-import ^
  --tcp-adapter ^
  --flag-generate-cmake ^
  --output-name=%OUTPUT_NAME% ^
  --library-interface

@echo off
popd