import logging

from src.migrationTool.migration_types import Config
from src.migrationTool.migration_types import Architecture
from src.migrationTool.gitMigration import GithubUploader

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

MIGRATE_DOCKER = True

config = Config("config.yaml")
architecture = Architecture.load_architecture("architecture.yaml")

Uploader = GithubUploader(config, architecture)
if MIGRATE_DOCKER:
    Uploader.docker_image_migration_monorepo()
# Uploader.upload_mono_repo("MontiCore/EmbeddedMontiArc", secrets, )
Uploader.upload_mono_repo()
