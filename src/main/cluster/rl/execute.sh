trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
nohup roscore &
sleep 2

echo "Start up environment..."
nohup python bin/ros-gym/launcher.py --environment \"TopoEnv\" --play --constraint --verbose --render 1 &
sleep 2

oldpath=$(pwd)
. $oldpath/config.sh
cd "${BINARY}" 
nohup ./agent &
sleep 2
wait
