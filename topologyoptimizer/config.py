import configparser
from pathlib import Path
from os.path import exists

def load_config():
    config = configparser.ConfigParser()

    current_path = Path( __file__ ).parent.resolve()
    project_root_dir = current_path.joinpath("../").resolve()
    resources_dir = project_root_dir.joinpath("resources").resolve()
    config_file = project_root_dir.joinpath("config.ini").resolve()

    if not exists(config_file):
        raise Exception("No config.ini file could be found. Please create one by copying the template config.ini.template to config.ini in the root folder of the project")
    
    with open(config_file) as config_file:
        config.read_file(config_file)

    # set dynamic settings
    config.set("DEFAULT", "ProjectRootDirectory", str(project_root_dir))
    config.set("DEFAULT", "ResourcesDirectory", str(resources_dir))

    return config


config = load_config()