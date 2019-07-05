#!/bin/bash
if [ -z "$1" ]; then 
	TARGET="Release"; 
else 
	TARGET="$1"; 
fi

if [ ! -d "$TARGET" ]; then
  mkdir $TARGET
fi
echo Building target ${TARGET}...
pushd $TARGET
cmake -DCMAKE_BUILD_TYPE=$TARGET -G "Unix Makefiles" ../
make
popd