docker exec -it carlacomponents /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/ && rm -rf Bumpbot/target && mvn clean install -pl Bumpbot -s settings.xml && Bumpbot/target/middleware/bumper_bumpBot/build/bumper_bumpBot/coordinator/Coordinator_bumper_bumpBot'
# (c) https://github.com/MontiCore/monticore  

