#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#


apt update
apt --assume-yes install libxext6
apt --assume-yes install libxrender1
apt --assume-yes install libxtst6

LIBGCC=$(find /lib -name libgcc* -print -quit)
echo LIBGCC: $LIBGCC
LIBGCC_FOLDER=$(dirname ${LIBGCC})
echo LIBGCC_FOLDER: $LIBGCC_FOLDER
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${LIBGCC_FOLDER}