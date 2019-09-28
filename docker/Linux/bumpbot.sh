docker exec -it carlacomponents /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Bumpbot/ && rm -rf target && mvn clean install -s settings.xml && Bumpbot/target/middleware/bumper_bumpBot/build/bumper_bumpBot/coordinator/Coordinator_bumper_bumpBot'

