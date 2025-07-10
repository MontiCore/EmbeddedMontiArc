#
# (c) https://github.com/MontiCore/monticore
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
pushd $DIR
mvn clean install -s settings.xml $*
popd