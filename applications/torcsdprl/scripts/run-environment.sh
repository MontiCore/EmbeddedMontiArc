xterm -title "ROSCORE" -e "roscore; bash" &
sleep 2

#python ros-gym-torcs/launcher.py
cd aux
xterm -title "ENVIRONMENT" -e "python ros-gym-torcs/launcher_p3.py" &
