import configparser
from pathlib import Path
from os.path import exists

def load_config():
    config = configparser.ConfigParser()

    current_path = Path( __file__ ).parent.resolve()
    config_file_path = current_path.joinpath("../../config.ini").resolve()

    if not exists(config_file_path):
        raise Exception("No config.ini file could be found. Please create one by copying the template config.ini.template to config.ini in the root folder of the project")
    
    with open(config_file_path) as config_file:
        config.read_file(config_file)

    return config


config = load_config()