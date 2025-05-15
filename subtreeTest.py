import logging
import os.path
import subprocess

from tqdm import tqdm

import Config
import sourceAnalysis
from repoMigration import GithubUploader
# from UploaderTest import architecture
from sourceAnalysis import findLargeFilesInHistory
from pipelineMigration import GitlabToGithub, GitlabToGithubSubtree
from sourceAnalysis import run_git_filter_repo, split_large_files

import yaml
import git

from sourceAnalysis.repoCleaning import remove_lfs, remove_lfs_from_gitattributes

SPLIT_LARGE_FILES = True
REMOVE_LFS = True
REMOVE_LARGE_FILES = True

logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S')
logger = logging.getLogger(__name__)

logger.info("Starting scan and clone")
config = Config.Config("config.yaml")
dr = sourceAnalysis.clone_and_scan(config)

input("Please adapt the architecture.yaml file and press enter to continue")

data = yaml.safe_load(open("architecture.yaml"))
# Get number of iterations for progress bar
numberIterations = 0
for repoID in data.keys():
    if data[repoID]["Branches"] is None:
        numberIterations += len(data[repoID]["StaleBranches"])
    elif data[repoID]["StaleBranches"] is None:
        numberIterations[str(repoID)] = len(data[repoID]["Branches"])
    else:
        numberIterations += len(set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))

with tqdm(total=numberIterations, desc="Migrating repository branches", unit="branches") as progress:
    for repoID in data.keys():
        repo_path = os.path.join(os.getcwd(), "repos", data[repoID]["Name"])
        repo = git.Repo(repo_path)

        # Check, which branches have to be migrated for the repo
        if data[repoID]["Branches"] is None:
            branches = data[repoID]["StaleBranches"]
        elif data[repoID]["StaleBranches"] is None:
            branches = data[repoID]["Branches"]
        else:
            branches = list(set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))

        if REMOVE_LFS:
            # Check if the repo uses LFS

            lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=repo_path,
                                       capture_output=True,
                                       text=True)
            # If yes remove LFS
            if lfs_check.stdout:
                remove_lfs(repo_path)

        for branch in branches:
            repo.git.checkout(branch)
            if SPLIT_LARGE_FILES:
                split_large_files(repo_path)

            if REMOVE_LFS:
                remove_lfs_from_gitattributes(repo_path)
            progress.update(1)
        repo.git.checkout("master")

if REMOVE_LARGE_FILES:
    for repoID in tqdm(data.keys(), desc="Cleaning large files, this may take a while", unit="repos"):
        repo_path = os.path.join(os.getcwd(), "repos", data[repoID]["Name"])
        run_git_filter_repo(repo_path)

# Build monorepo from cleaned repos
Uploader = GithubUploader.GithubUploader(config)
Uploader.add_repos_as_subtree(config.monorepoName, config.monorepoNamespace, data.keys())

# Convert pipelines
prefix = {}  # Path to each subtree in the monorepo
monorepoNamespace = config.monorepoNamespace.split("/")
for repoID in data.keys():
    repoNamespace = ",".join([i for i in data[repoID]["Namespace"].split("/") if i not in monorepoNamespace])
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
