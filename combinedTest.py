import subprocess

from tqdm import tqdm

import Config
import sourceAnalysis
#from UploaderTest import architecture
from sourceAnalysis import findLargeFilesInHistory
from pipelineMigration import GitlabToGithub
from sourceAnalysis import run_git_filter_repo, split_large_files

import yaml
import git

from sourceAnalysis.repoCleaning import remove_lfs, remove_lfs_from_gitattributes

print("Starting scan and clone")

config = Config.Config("config.yaml")
dr = sourceAnalysis.scanAndCloneRepos(config)

data = yaml.safe_load(open("architecture.yaml"))
#for repoID in data.keys():
    #print()
    #print(data[repoID]["Name"])
    #findLargeFilesInHistory("repos/" + data[repoID]["Name"])

    #split_large_files("./repos/" + data[repoID]["Name"])
    #run_git_filter_repo("repos/" + data[repoID]["Name"])
    #findLargeFilesInHistory("repos/" + data[repoID]["Name"])

#print()
#print("Starting migration")
#for repoID in tqdm(data.keys(), desc="Migrating pipelines"):
#    gitlabRepoPath = "repos/" + data[repoID]["Name"]
#    githubRepoPath = "repos/" + data[repoID]["Name"]
#    print(repoID)
#    GitlabToGithub(gitlabRepoPath, githubRepoPath, "pipeline", ["GITLABTOKEN", "CI_API_V4_URL", "CI_PROJECT_ID"])

print()
print("Starting migration")
architecture = yaml.safe_load(open("architecture.yaml"))
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
        split_large_files("./repos/" + data[repoID]["Name"])
        print("Migrating branch: " + branch)
        print(repoID)
        remove_lfs_from_gitattributes("./repos/" + data[repoID]["Name"])

        GitlabToGithub(gitlabRepoPath, githubRepoPath, "pipeline", ["GITLABTOKEN", "CI_API_V4_URL", "CI_PROJECT_ID"])
        #input("WAIT")


    repo.git.checkout("master")
    run_git_filter_repo("repos/" + data[repoID]["Name"])
