#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#


SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh

if [ -z "$1" ]; then 
	TARGET="Release"; 
else 
	TARGET="$1"; 
fi

if [ ! -d "build" ]; then
  mkdir build
fi
print "Building with target ${TARGET}..."
pushd build
cmake -DCMAKE_BUILD_TYPE=$TARGET -G "Unix Makefiles" ../
print "Compiling..."
make
popd
