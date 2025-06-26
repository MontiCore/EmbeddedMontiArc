import logging
from src.migrationTool.migration_types import Config
from src.migrationTool.migration_types import Architecture
from src.migrationTool.pipelineMigration import GitlabToGithubSubtree

"""
This scrip is used to convert the Gitlab CI pipelines to Github Actions pipelines in the monorepo.
"""

REBUILD_LARGE_FILES = False  # Set to true if large files have been split

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

config = Config("config.yaml")
architecture = Architecture.load_architecture("architecture.yaml")

# Convert pipelines
GitlabToGithubSubtree(architecture.repoIDs, architecture, config, rebuild=REBUILD_LARGE_FILES)
