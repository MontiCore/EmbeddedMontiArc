import logging
import os.path

from src.types import Config
from src.pipelineMigration import GitlabToGithubSubtree

import yaml

SPLIT_LARGE_FILES = False
REMOVE_LFS = True
REMOVE_LARGE_FILES = True

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

config = Config.Config("../config.yaml")
data = yaml.safe_load(open("../architecture.yaml"))

# Convert pipelines
prefix = {}  # Path to each subtree in the monorepo
monorepoNamespace = config.monorepoNamespace.split("/")
for repoID in data.keys():
    repoNamespace = "/".join([i for i in data[repoID]["Namespace"].split("/") if i not in monorepoNamespace])
    prefix[data[repoID]["Name"]] = os.path.join(repoNamespace, data[repoID]["Name"])

# Get secrets
secrets = {}
for repoID in data.keys():
    secrets[data[repoID]["Name"]] = []
    for name, secret in data[repoID]["Secrets"].items():
        if secret["Value"] == "Please add a value":
            continue
        if secret["Secret"].lower() == "y":
            secrets[data[repoID]["Name"]].append(name)
        elif secret["Secret"].lower() == "e":
            secrets[data[repoID]["Name"]].append((name, "${{ secrets." + str(secret["Value"]) + " }}"))
        else:
            secrets[data[repoID]["Name"]].append((name, secret["Value"]))
monorepo_path = os.path.join(os.getcwd(), "repos", config.monorepoName)

GitlabToGithubSubtree(data.keys(), data, config, monorepo_path, prefix, secrets, rebuild=SPLIT_LARGE_FILES)
