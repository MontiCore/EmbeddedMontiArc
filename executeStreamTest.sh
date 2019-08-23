#!/bin/bash
# (c) https://github.com/MontiCore/monticore  

# exit with code on error
set -e

cd native
source variables.sh
cd "${PROJECT_ROOT}/${1}"
./TestsForCurrentModel
