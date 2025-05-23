import logging

from src.migrationTool.migration_types import Config
from src.migrationTool.gitMigration.GitlabDownloader import GitlabDownloader

"""
This script is used to clone and scan GitLab repositories. It creates the architecture.yaml file for the later steps.
"""

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

logger.info("Starting scan and clone")
config = Config("config.yaml")
gitlab_downloader = GitlabDownloader(config)
gitlab_downloader.clone()
gitlab_downloader.scan()
