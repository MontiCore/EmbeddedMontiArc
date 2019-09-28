docker exec -it carlacomponents /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/ && rm -rf Autopilot/target && mvn clean install -pl Autopilot -s settings.xml && Autopilot/target/build/test_autopilot/coordinator/Coordinator_test_autopilot'

