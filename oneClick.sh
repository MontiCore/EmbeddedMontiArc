curDir=$(readlink -f `dirname $0`)
rm -rf "$curDir/target"
./generate.sh
./compile.sh
./startAll.sh