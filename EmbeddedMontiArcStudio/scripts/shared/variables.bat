@rem (c) https://github.com/MontiCore/monticore  
pushd %~dp0
cd ..
cd ..
set HOME=%CD%
popd
set SIMULATION_URL=http://localhost:8080/
set JAVA_HOME=%HOME%\jdk
set TOMCAT_HOME=%HOME%\apache-tomcat-9.0.5
set MINGW_HOME=%HOME%\mingw64
set OCTAVE_HOME=%HOME%\octave-4.2.1
set ARMADILLO_HOME=%HOME%\armadillo
set CPP_DIR=%HOME%\cpp
set DLL_DIR=%HOME%\dll
set LIB_DIR=%HOME%\lib
set EXTERNAL_LIBS=%HOME%\externallibs
set PATH=%EXTERNAL_LIBS%;%LIB_DIR%;%MINGW_HOME%\bin;%DLL_DIR%;%JAVA_HOME%\bin;%OCTAVE_HOME%\bin;%PATH%
set SVG_HOME=%HOME%\visualisation
set REPORTING_HOME=%HOME%\reporting
set CHROME=%HOME%\chrome\GoogleChromePortable.exe
set TESTS_CPP_DIR=%HOME%\tests-cpp
set TEST_EXEC_DIR=%HOME%\exec
set ARMADILLO_HOME=%HOME%\armadillo-8.200.2
REM set CLUSTERER_CPP_DIR=%HOME%\clusterer-cpp
set CLUSTERER_EXEC_DIR=%HOME%\exec
set VERIFICATION_HOME=%HOME%\viewverification
set DISTR_SIM=%HOME%\distributed-simulator
set PSQL=%DISTR_SIM%\PostgreSQLPortable
set SFS=%DISTR_SIM%\SmartFoxServer_2X\SFS2X
set RMI=%DISTR_SIM%\RMIModelServer
set RMI_PATH=%EXTERNAL_LIBS%;%LIB_DIR%;%MINGW_HOME%\bin;%DLL_DIR%;%JAVA_HOME%\bin;%OCTAVE_HOME%\bin;
set WASM_HOME=%HOME%\emam2wasm
set PACMAN_HOME=%HOME%\pacman
