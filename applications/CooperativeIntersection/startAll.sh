#!/bin/bash
# (c) https://github.com/MontiCore/monticore  
source /opt/ros/kinetic/setup.bash

if [ -z "$COINCAR_SIM_HOME" ]
then
    echo "Environment variable COINCAR_SIM_HOME is not set. Set it to the base of your coincar_sim installation."
    exit 1
fi

if [ ! -x $COINCAR_SIM_HOME/startFramework.sh ]
then
    echo "Can not find $COINCAR_SIM_HOME/startFramework.sh. Your coincar_sim installation seems to be broken."
    exit 1
fi

# sim framework
$COINCAR_SIM_HOME/startFramework.sh &
sleep 5

# Components
# execution delay in ms
exeDelay=200

(./target/build/ba_system_intersectionController/coordinator/Coordinator_ba_system_intersectionController -t $exeDelay) &
(./target/build/ba_system_velocityController_2_/coordinator/Coordinator_ba_system_velocityController_2_ -t $exeDelay) &
(./target/build/ba_system_stopCommQuality_1_/coordinator/Coordinator_ba_system_stopCommQuality_1_ -t $exeDelay) &
(./target/build/ba_system_velocityController_1_/coordinator/Coordinator_ba_system_velocityController_1_ -t $exeDelay) &
(./target/build/ba_system_stopCommQuality_2_/coordinator/Coordinator_ba_system_stopCommQuality_2_ -t $exeDelay) &
(./target/build/ba_system_collisionDetection/coordinator/Coordinator_ba_system_collisionDetection -t $exeDelay) &
wait
