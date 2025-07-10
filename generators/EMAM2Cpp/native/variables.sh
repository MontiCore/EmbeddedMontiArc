#!/bin/bash
# (c) https://github.com/MontiCore/monticore  


pushd `pwd` > /dev/null
cd ..
export PROJECT_ROOT="`pwd`"
export HOME="${PROJECT_ROOT}/native"
popd > /dev/null
export SIMULATION_URL="http://localhost:8080/"
export JAVA_HOME="${HOME}/jdk"
export TOMCAT_HOME="${HOME}/apache-tomcat-9.0.5"
export MINGW_HOME="${HOME}/mingw64"
export OCTAVE_HOME="${HOME}/octave-4.2.1"
export ARMADILLO_HOME="${HOME}/armadillo_linux"
export CPP_DIR="${HOME}/cpp"
export DLL_DIR="${HOME}/dll"
export LIB_DIR="${HOME}/lib"
export EXTERNAL_LIBS="${HOME}/externallibs"
export PATH="${EXTERNAL_LIBS}:${LIB_DIR}:${MINGW_HOME}/bin:${DLL_DIR}:${JAVA_HOME}/bin:${OCTAVE_HOME}/bin:${PATH}"
export SVG_HOME="${HOME}/visualisation"
export REPORTING_HOME="${HOME}/reporting"
export CHROME="${HOME}/chrome/GoogleChromePortable.exe"
export TEST_EXEC_DIR="${HOME}/exec"
# export CLUSTERER_CPP_DIR="${HOME}/clusterer-cpp"
export CLUSTERER_EXEC_DIR="${HOME}/exec"
export VERIFICATION_HOME="${HOME}/viewverification"
export NFPVERIFICATION_HOME="${HOME}/nfpverification"
export DISTR_SIM="${HOME}/distributed-simulator"
export PSQL="${DISTR_SIM}/PostgreSQLPortable"
export SFS="${DISTR_SIM}/SmartFoxServer_2X/SFS2X"
export RMI="${DISTR_SIM}/RMIModelServer"
export RMI_PATH="${EXTERNAL_LIBS}:${LIB_DIR}:${MINGW_HOME}/bin:${DLL_DIR}:${JAVA_HOME}/bin:${OCTAVE_HOME}/bin:"
export WASM_HOME="${HOME}/emam2wasm"
export PACMAN_HOME="${HOME}/pacman"
export SUPERMARIO_HOME="${HOME}/supermario"
export CNCVERIFICATION_HOME="${HOME}/cncverification"
