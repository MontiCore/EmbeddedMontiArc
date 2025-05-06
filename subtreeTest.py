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
print("Starting migration")
data = yaml.safe_load(open("architecture.yaml"))
for repoID in tqdm(data.keys(), desc="Migrating pipelines"):
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
        print("Migrating branch: " + branch)
        print(repoID)
        remove_lfs_from_gitattributes("./repos/" + data[repoID]["Name"])

        #input("WAIT")


    repo.git.checkout("master")
    #run_git_filter_repo("repos/" + data[repoID]["Name"])

Uploader = GithubUploader.GithubUploader(config.targetToken, config.sourceToken)

#Uploader.initRepo("subtreeTest")
#Uploader.addSubtree("subtreeTest", "EMADL2CPP", "generator")
#Uploader.addSubtree("subtreeTest", "MNISTCalculator", "application")
Uploader.addReposAsSubtree("subtreeTest", data.keys())
split_large_files("./repos/subtreeTest")
#ToDo: Why are the files being recreated being concatenated in the wrong order partab + partaa?
run_git_filter_repo("./repos/subtreeTest")

gitlabRepoPath = [("./repos/"+ data[repoID]["Name"],repoID) for repoID in data.keys()]

prefix = {}
for repoID in data.keys():
    prefix[data[repoID]["Name"]] = data[repoID]["Namespace"]+"/"+data[repoID]["Name"]

secrets = {}
for repoID in data.keys():
    secrets[data[repoID]["Name"]] = ["GITLABTOKEN", "CI_API_V4_URL", "CI_PROJECT_ID"]

GitlabToGithubSubtree(gitlabRepoPath, "./repos/subtreeTest",prefix , secrets)