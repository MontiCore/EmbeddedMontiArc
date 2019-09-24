#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore.
#


SCRIPTS_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
#Import print()
. $SCRIPTS_DIR/print.sh
ROOT_DIR="$SCRIPTS_DIR/../.."
LIBS_DIR=$ROOT_DIR/hardware_emulator/libs/unix
if [ ! -d "$LIBS_DIR" ]; then
  mkdir $LIBS_DIR
fi

if [ -z "$1" ]; then 
	DEP_TARGET="Release"; 
else 
	DEP_TARGET="$1"; 
fi

pushd $ROOT_DIR


print "Building dependency: pe-parse..."
pushd pe-parse
$SCRIPTS_DIR/build_compile.sh $DEP_TARGET
print "Copying pe-parse library..."
cp "build/pe-parser-library/libpe-parser-library.a" "$LIBS_DIR"
popd


echo ""
print "Building dependency: Zydis..."
pushd zydis
$SCRIPTS_DIR/build_compile.sh $DEP_TARGET
print "Copying Zydis library..."
cp "build/libZydis.a" "$LIBS_DIR"
popd


echo ""
print "Building dependency: Unicorn..."
pushd unicorn
./make.sh
print "Copying Unicorn library..."
cp "libunicorn.a" "$LIBS_DIR"
popd

# echo ""
# echo "**************************************"
# echo "      Making Zydis Release/Debug"
# echo "**************************************"
# pushd zydis
# pushd dependencies/zycore
# ../../../scripts/build.sh Release
# ../../../scripts/build.sh Debug
# popd
# ../scripts/build.sh Release
# ../scripts/build.sh Debug
# popd

popd
