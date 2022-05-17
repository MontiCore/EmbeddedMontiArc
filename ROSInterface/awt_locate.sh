#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#


JAWT_LIB=$(find / -name libjawt.so -print -quit)
AWT_LIB=$(find / -name libawt.so -print -quit)
echo JAWT_LIB: $JAWT_LIB
echo AWT_LIB: $AWT_LIB
JAWT_FOLDER=$(dirname ${JAWT_LIB})
AWT_FOLDER=$(dirname ${AWT_LIB})
echo JAWT_FOLDER: $JAWT_FOLDER
echo AWT_FOLDER: $AWT_FOLDER
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:${JAWT_FOLDER}:${AWT_FOLDER}