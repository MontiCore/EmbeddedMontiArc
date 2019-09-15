#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
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

pushd $ROOT_DIR/samples

print "Updating simple sample..."
pushd simple
make
print "Copying simple sample..."
cp "sample_simple.so" "../../hardware_emulator/bin/"
popd


print "Updating funccalling sample..."
pushd funccalling
make
print "Copying funccalling sample..."
cp "sample_functioncalling.so" "../../hardware_emulator/bin/"
popd

print "Updating syscall sample..."
pushd syscall_so
make
print "Copying syscall sample..."
cp "sample_syscall.so" "../../hardware_emulator/bin/"
popd

popd
