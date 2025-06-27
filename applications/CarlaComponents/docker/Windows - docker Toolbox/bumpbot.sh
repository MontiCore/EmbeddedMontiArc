winpty docker exec \
# (c) https://github.com/MontiCore/monticore  
-it carlacomponents \
bash \
-c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Bumpbot/ && rm -rf target && cd .. && mvn clean install -pl Bumpbot -s settings.xml && Bumpbot/target/middleware/bumper_bumpBot/build/bumper_bumpBot/coordinator/Coordinator_bumper_bumpBot'
