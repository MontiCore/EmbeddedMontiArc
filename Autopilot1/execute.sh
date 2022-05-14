trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
sleep 2 
oldpath=$(pwd)
. $oldpath/config.sh
cd "${BINARY}" 
./agent -t 100 &
sleep 2
cd "${SIMULATOR_PATH}"
java -jar basic-simulator.jar
cd $oldpath

