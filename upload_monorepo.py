import logging

import yaml
from github.Migration import Migration

from src import Config
from src.Architecture import Architecture
from src.gitMigration import GithubUploader

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

MIGRATE_DOCKER = True

config = Config.Config("config.yaml")
architecture = Architecture.load_architecture("architecture.yaml")

Uploader = GithubUploader.GithubUploader(config,architecture)
if MIGRATE_DOCKER:
    Uploader.docker_image_migration_monorepo()
#Uploader.upload_mono_repo("MontiCore/EmbeddedMontiArc", secrets, )
Uploader.upload_mono_repo()