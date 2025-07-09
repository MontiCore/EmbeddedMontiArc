curDir=$(readlink -f `dirname $0`)
# (c) https://github.com/MontiCore/monticore  
rm -rf "$curDir/target"
./generate.sh
./compile.sh
./startAll.sh
