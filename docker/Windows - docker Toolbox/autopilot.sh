winpty docker exec \
# (c) https://github.com/MontiCore/monticore  
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Autopilot/ && rm -rf target && cd .. && mvn clean install -pl Autopilot -s settings.xml && Autopilot/target/build/test_autopilot/coordinator/Coordinator_test_autopilot'
