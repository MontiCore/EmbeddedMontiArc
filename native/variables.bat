@rem (c) https://github.com/MontiCore/monticore  

pushd %~dp0
cd ..
set PROJECT_ROOT=%CD%
set HOME=%PROJECT_ROOT%\native
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
set TEST_EXEC_DIR=%HOME%\exec
REM set CLUSTERER_CPP_DIR=%HOME%\clusterer-cpp
set CLUSTERER_EXEC_DIR=%HOME%\exec
set VERIFICATION_HOME=%HOME%\viewverification
set NFPVERIFICATION_HOME=%HOME%\nfpverification
set DISTR_SIM=%HOME%\distributed-simulator
set PSQL=%DISTR_SIM%\PostgreSQLPortable
set SFS=%DISTR_SIM%\SmartFoxServer_2X\SFS2X
set RMI=%DISTR_SIM%\RMIModelServer
set RMI_PATH=%EXTERNAL_LIBS%;%LIB_DIR%;%MINGW_HOME%\bin;%DLL_DIR%;%JAVA_HOME%\bin;%OCTAVE_HOME%\bin;
set WASM_HOME=%HOME%\emam2wasm
set PACMAN_HOME=%HOME%\pacman
set SUPERMARIO_HOME=%HOME%\supermario
set CNCVERIFICATION_HOME=%HOME%\cncverification
