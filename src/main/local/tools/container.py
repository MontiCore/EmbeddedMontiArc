# This file is only for local execution of the preprocessing!
import os
import pathlib
import sys

project_path = pathlib.Path().resolve().parent  # Project path

if sys.version_info > (3, 9, 100):
    sys.exit("Sorry, Python > 3.9 is not supported")

import docker

client = docker.from_env()

container = client.api.create_container(
    image="preprocessing", command=["python3", "create_h5_from_raw.py"], working_dir="/preprocessing",
    volumes=['/preprocessing'],
    host_config=client.api.create_host_config(binds=[r'C:\Users\Anis\Desktop\toolchain\preprocessing:/preprocessing'])
)
container_id = container.get('Id')  # get container ID
client.api.start(container=container_id)  # start container
container_obj = client.containers.get(container_id)  # get container object

for line in container_obj.logs(stream=True):  # print out logs
    print(line.strip())

result = client.api.wait(container=container_id)  # save container exit status
client.containers.prune()  # delete container after execution
