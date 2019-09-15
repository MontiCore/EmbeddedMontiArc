#!/bin/bash
#
#
#  ******************************************************************************
#  MontiCAR Modeling Family, www.se-rwth.de
#  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
#  All rights reserved.
#
#  This project is free software; you can redistribute it and/or
#  modify it under the terms of the GNU Lesser General Public
#  License as published by the Free Software Foundation; either
#  version 3.0 of the License, or (at your option) any later version.
#  This library is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
#  Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public
#  License along with this project. If not, see <http://www.gnu.org/licenses/>.
# *******************************************************************************
#

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
