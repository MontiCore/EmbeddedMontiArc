docker exec -it emam2carla /bin/bash -c 'source /opt/ros/kinetic/setup.bash && cd /usr/src/project/ && java -jar ../mw-generator.jar valid.json && target/compile.sh && build/test_bumpBot/coordinator/Coordinator_test_bumpBot'

