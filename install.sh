#!/bin/bash
#
# (c) https://github.com/MontiCore/monticore
#
# The license generally applicable for this project
# can be found under https://github.com/MontiCore/monticore
#


# Make sure the script runs from its original location
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR

mvn clean install -s settings.xml $*

popd