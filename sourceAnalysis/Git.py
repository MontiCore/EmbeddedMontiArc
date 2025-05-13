from abc import ABC
import yaml
import os


class Git(ABC):
    def __init__(self):
        pass

    # ToDo: Move implementation
    @staticmethod
    def create_config_file():
        """
        Creates a configuration file for the Migration Tool
        """
        if not os.path.exists("config.yaml"):
            data = {"URL": "Please add the URL of the Git instance",
                    "SourceToken": "Please add the private token of the source Git instance",
                    "TargetToken:": "Please add the private token of the target Git instance",
                    "Repos": "Please add the repo IDs for migration"}
            yaml.safe_dump(data, open("config.yaml", 'w'))
        else:
            print("config.yaml already exists.")
