#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

# exit with code on error
set -e

cd native
source variables.sh
source compileTestsArmadilloBackendOpenBLAS.sh "${PROJECT_ROOT}/$1" "${PROJECT_ROOT}/$2"
