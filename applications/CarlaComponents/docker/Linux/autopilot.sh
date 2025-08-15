docker exec -it carlacomponents /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/ && rm -rf Autopilot/target && mvn clean install -pl Autopilot -s settings.xml && Autopilot/target/middleware/wrapper_wrapper/build/wrapper_wrapper/coordinator/Coordinator_wrapper_wrapper'
# (c) https://github.com/MontiCore/monticore  
