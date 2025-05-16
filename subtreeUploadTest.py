import logging

import yaml
from repoMigration import GithubUploader
import Config

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

config = Config.Config("config.yaml")
architecture = yaml.safe_load(open("architecture.yaml"))

# config.monorepoName = "EMA2"

dockerImages = []
secrets = {}
for reopId in architecture.keys():
    if "DockerImages" in architecture[reopId] and architecture[reopId]["DockerImages"] is not None:
        dockerImages.append(architecture[reopId]["DockerImages"])
    for secretName, secretValues in architecture[reopId]["Secrets"].items():
        if secretValues["Secret"].lower() == "y":
            if secretName not in secrets.keys() and secretValues["Value"] != "Please add a value":
                secrets[secretName] = secretValues["Value"]

Uploader = GithubUploader.GithubUploader(config)
if dockerImages:
    Uploader.docker_image_migration_monorepo(architecture)
Uploader.upload_mono_repo("MontiCore/EmbeddedMontiArc", secrets, )
