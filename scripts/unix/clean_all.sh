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


print "Cleaning all build files..."

pushd $ROOT_DIR

print "Cleaning pe-parse"
pushd pe-parse
rm -r build
popd

print "Cleaning Zydis"
pushd zydis
rm -r build
# pushd dependencies/zycore
# rm -r build
# popd
popd

print "Cleaning Unicorn"
pushd unicorn
make clean
popd

print "Cleaning hardware_emulator"
pushd hardware_emulator
rm -r build
popd

popd
