xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

#python ros-gym-torcs/launcher.py
cd aux
python ros-gym-torcs-python2/launcher.py
