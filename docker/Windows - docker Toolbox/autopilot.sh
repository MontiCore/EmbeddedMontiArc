winpty docker exec \
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Autopilot/ && rm -rf target && cd .. && mvn clean install -s settings.xml && Autopilot/target/build/test_autopilot/coordinator/Coordinator_test_autopilot'