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
pushd $DIR/../samples

echo "**************************************"
echo "        Updating simple sample"
echo "**************************************"
pushd simple
make
cp "sample_simple.so" "../../hardware_emulator/bin/"
popd


echo "**************************************"
echo "     Updating funccalling sample"
echo "**************************************"
pushd funccalling
make
cp "sample_functioncalling.so" "../../hardware_emulator/bin/"
popd

echo "**************************************"
echo "      Updating syscall sample"
echo "**************************************"
pushd syscall_so
make
cp "sample_syscall.so" "../../hardware_emulator/bin/"
popd

popd
