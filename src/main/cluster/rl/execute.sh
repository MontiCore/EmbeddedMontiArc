trap 'kill $(jobs -pr)' SIGINT SIGTERM EXIT
roscore &
sleep 2
xterm -title "Rosgym" -e "rostopic echo /topo/reset; bash" &
sleep 2
echo "Start up environment..."
xterm -title "Gym-Environment" -e "python bin/ros-gym/launcher.py --environment \"TopoEnv\" --play; bash" &
sleep 2

oldpath=$(pwd)
. $oldpath/config.sh
cd "${BINARY}" 
./agent -t 100 &
sleep 2
