docker exec -it emam2carla /bin/bash -c 'export PYTHONPATH=/opt/carla/PythonAPI/carla/dist/carla-0.9.5-py2.7-linux-x86_64.egg:/opt/carla/PythonAPI && source /opt/ros/kinetic/setup.bash && cd /usr/src/emam2carla/EMAM2Carla/ && rm -rf target && java -jar ../mw-generator.jar valid.json && target/compile.sh && target/build/test_bumpBot/coordinator/Coordinator_test_bumpBot'
# (c) https://github.com/MontiCore/monticore  

