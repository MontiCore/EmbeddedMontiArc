winpty docker exec \
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Bumpbot/ && rm -rf target && cd .. && java -jar mw-generator.jar Bumpbot/Bumpbot.json && Bumpbot/target/compile.sh && Bumpbot/target/build/test_bumpBot/coordinator/Coordinator_test_bumpBot'