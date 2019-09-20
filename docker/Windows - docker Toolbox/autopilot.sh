winpty docker exec \
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Autopilot/ && rm -rf target && java -jar ../mw-generator.jar Autopilot/Autopilot.json && target/compile.sh && target/build/test_autopilot/coordinator/Coordinator_test_autopilot'