import os

import yaml


class Config:
    def __init__(self, path):
        data = yaml.safe_load(open(path))
        self.url = data['URL']
        self.sourceToken = data['SourceToken']
        self.sourceUser = data['SourceUser']
        self.targetToken = data['TargetToken']
        self.targetUser = data['TargetRepoOwner']
        self.repoIDS = [str(i) for i in data['Repos']]
        self.monorepoName = data['MonorepoName']
        self.monorepoNamespace = data['MonorepoNamespace']

    @staticmethod
    def create_config_file():
        """
        Creates a configuration file for the Migration Tool
        """
        if not os.path.exists("../config.yaml"):
            data = {"URL": "Please add the URL of the source Git instance",
                    "SourceToken": "Please add the private token of the source Git instance",
                    "TargetToken:": "Please add the private token of the target Git instance",
                    "Repos": "Please add the repo IDs for migration",
                    "MonorepoName": "Please add the name of the monorepo",
                    "SourceUser": "Please add the name of the user on the source system",
                    "TargetRepoOwner": "Please add the name of the user on the target system",
                    "MonorepoNamespace": "Please add the namespace of the monorepo, this removed from the begining of the repo namespaces"}

            yaml.safe_dump(data, open("../config.yaml", 'w'))
        else:
            print("config.yaml already exists.")

    def write_migrated_docker_images(self, dockerImages):
        """
        Write the migrated docker images to the config file
        :param dockerImages: List of migrated docker images
        """
        data = yaml.safe_load(open("../config.yaml"))
        data["MigratedDockerImages"] = dockerImages
        yaml.safe_dump(data, open("../config.yaml", 'w'))

    def read_migrated_docker_images(self):
        """
        Read the migrated docker images from the config file
        :return: List of migrated docker image URLs
        """

        data = yaml.safe_load(open("../config.yaml"))
        if "MigratedDockerImages" in data.keys():
            return data["MigratedDockerImages"]
        else:
            return []
