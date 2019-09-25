winpty docker exec \
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Autopilot/ && rm -rf target && cd .. && java -jar mw-generator.jar Autopilot/Autopilot.json && Autopilot/target/compile.sh && Autopilot/target/build/test_autopilot/coordinator/Coordinator_test_autopilot'