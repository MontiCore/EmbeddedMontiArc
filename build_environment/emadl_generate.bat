@echo off

set Help=0
if "%1"=="--help" set Help=1
if [%1] == [] set Help=1
if [%Help%] == [1] (
    echo Usage:
    echo     emadl_generate.bat 'folder'
    echo Description:
    echo     Generates the CPP files for the EMADL project specified in the 'folder'/ema_config file
    echo Configuration:
    echo     The 'EMADL_TO_CPP_PROJECT' and 'EMADL_TO_CPP_VERSION' properties must be set inside the 'config' file
    exit /B 0
)

pushd %~dp0

call get_config.bat config
call get_config.bat %1/ema_config

set EMADL_TO_CPP_JAR=%EMADL_TO_CPP_PROJECT%\target\embedded-montiarc-emadl-generator-%EMADL_TO_CPP_VERSION%-jar-with-dependencies.jar

cd %1

set OUTPUT_DIR=target/cpp

@echo on

java -cp %EMADL_TO_CPP_JAR% de.monticore.lang.monticar.emadl.generator.EMADLGeneratorCli ^
  --models-dir=%MODEL_DIR% ^
  --root-model=%MODEL_NAME% ^
  --output-dir=%OUTPUT_DIR% ^
  --flag-use-armadillo-backend ^
  --flag-generate-cmake ^
  --armadillo-import ^
  --tcp-adapter ^
  --output-name=%OUTPUT_NAME%
:: --dyn-interface ^

@echo off
popd