import subprocess

from tqdm import tqdm

import Config
import sourceAnalysis
from repoMigration import GithubUploader
#from UploaderTest import architecture
from sourceAnalysis import findLargeFilesInHistory
from pipelineMigration import GitlabToGithub, GitlabToGithubSubtree
from sourceAnalysis import run_git_filter_repo, split_large_files

import yaml
import git

from sourceAnalysis.repoCleaning import remove_lfs, remove_lfs_from_gitattributes

print("Starting scan and clone")

config = Config.Config("config.yaml")
dr = sourceAnalysis.scanAndCloneRepos(config)
print()
data = yaml.safe_load(open("architecture.yaml"))
for repoID in tqdm(data.keys(), desc="Migrating pipelines", ):
    repo = git.Repo("./repos/" + data[repoID]["Name"])
    gitlabRepoPath = "repos/" + data[repoID]["Name"]
    githubRepoPath = "repos/" + data[repoID]["Name"]
    if data[repoID]["Branches"] is None:
        branches= data[repoID]["StaleBranches"]
    elif data[repoID]["StaleBranches"] is None:
        branches = data[repoID]["Branches"]
    else:
        branches = list(set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))
    lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd="repos/" + data[repoID]["Name"], capture_output=True,
                               text=True)
    if lfs_check.stdout:
        remove_lfs("./repos/" + data[repoID]["Name"])
    for branch in branches:
        repo.git.checkout(branch)
        #split_large_files("./repos/" + data[repoID]["Name"])
        #print("Migrating branch: " + branch)
        #print(repoID)
        remove_lfs_from_gitattributes("./repos/" + data[repoID]["Name"])

        #input("WAIT")


    repo.git.checkout("master")
    #run_git_filter_repo("repos/" + data[repoID]["Name"])

Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)
Uploader.addReposAsSubtree("subtreeTest", data.keys())

prefix = {}
for repoID in data.keys():
    prefix[data[repoID]["Name"]] = data[repoID]["Namespace"]+"/"+data[repoID]["Name"]

secrets = {}
for repoID in data.keys():
    secrets[data[repoID]["Name"]] = []
    for name,secret in data[repoID]["Secrets"].items():
        if secret["Value"] == "Please add a value":
            continue
        if secret["Secret"].lower() == "y":
            secrets[data[repoID]["Name"]].append(name)
        elif secret["Secret"].lower() == "e":
            secrets[data[repoID]["Name"]].append((name, "${{ secrets."+ str(secret["Value"]) +" }}"))
        else:
            secrets[data[repoID]["Name"]].append((name, secret["Value"]))
GitlabToGithubSubtree(data.keys(), data, config,  "./repos/subtreeTest",prefix , secrets)