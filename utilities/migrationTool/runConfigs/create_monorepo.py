import logging

from src.migrationTool.migration_types import Config, Architecture
from src.migrationTool.gitMigration import Git

"""
This script builds the monorepo from the cleaned repos.
"""

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

logger.info("Building monorepo")
config = Config("config.yaml")
architecture = Architecture.load_architecture("architecture.yaml")
git = Git()

# Build monorepo from cleaned repos
git.add_repos_as_subtree(config.monorepoName, config.monorepoNamespace, architecture)
