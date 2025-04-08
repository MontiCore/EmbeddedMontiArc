from tqdm import tqdm

import Config
import sourceAnalysis
from sourceAnalysis import findLargeFilesInHistory
from pipelineMigration import GitlabToGithub
from sourceAnalysis import run_git_filter_repo, split_large_files

import yaml


config = Config.Config("repos.yaml")
dr = sourceAnalysis.scanAndCloneRepos(config)

data = yaml.safe_load(open("architecture.yaml"))
for repoID in data.keys():
    print()
    print(data[repoID]["Name"])
    findLargeFilesInHistory("repos/" + data[repoID]["Name"])

    split_large_files("repos/" + data[repoID]["Name"], 1000000)
    run_git_filter_repo("repos/" + data[repoID]["Name"])
    #findLargeFilesInHistory("repos/" + data[repoID]["Name"])

print()
print("Starting migration")
for repoID in tqdm(data.keys(), desc="Migrating pipelines"):
    gitlabFilePath = "repos/" + data[repoID]["Name"] + "/.gitlab-ci.yml"
    githubFilePath = "repos/" + data[repoID]["Name"] + "/.main.yml"
    GitlabToGithub(gitlabFilePath, githubFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
