docker exec -it carlacomponents /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/carlacomponents/Bumpbot/ && rm -rf target && java -jar ../mw-generator.jar Bumpbot/Bumpbot.json && target/compile.sh && target/build/test_bumpBot/coordinator/Coordinator_test_bumpBot'

