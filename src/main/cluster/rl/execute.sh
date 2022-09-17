trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
nohup roscore &
sleep 2

echo "Start up environment..."
nohup python bin/ros-gym/launcher.py --environment \"TopoEnv\" --play &
sleep 2

oldpath=$(pwd)
. $oldpath/config.sh
cd "${BINARY}" 
nohup ./agent -t 100 &
sleep 2
wait
