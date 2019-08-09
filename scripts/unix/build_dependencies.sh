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

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR/..
echo ""
echo "**************************************"
echo "           Making Unicorn"
echo "**************************************"
pushd unicorn
./make.sh
popd

echo ""
echo "**************************************"
echo "      Making Zydis Release/Debug"
echo "**************************************"
pushd zydis
pushd dependencies/zycore
../../../scripts/build.sh Release
../../../scripts/build.sh Debug
popd
../scripts/build.sh Release
../scripts/build.sh Debug
popd

echo ""
echo "**************************************"
echo "    Making pe-parse Release/Debug"
echo "**************************************"
pushd pe-parse
../scripts/build.sh Release
../scripts/build.sh Debug
popd


cp "unicorn/libunicorn.a" "hardware_emulator/libs/Release/"
cp "unicorn/libunicorn.a" "hardware_emulator/libs/Debug/"
cp "zydis/Release/libZydis.a" "hardware_emulator/libs/Release/"
cp "zydis/Debug/libZydis.a" "hardware_emulator/libs/Debug/"

cp "pe-parse/Release/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Release/"
cp "pe-parse/Debug/pe-parser-library/libpe-parser-library.a" "hardware_emulator/libs/Debug/"

popd