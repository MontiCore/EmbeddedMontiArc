#
# (c) https://github.com/MontiCore/monticore
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR
java -jar basic-simulator.jar $*
popd