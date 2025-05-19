import subprocess

import git
from tqdm import tqdm

from src.Architecture import Architecture
from src.gitMigration.largeFiles import run_git_filter_repo
from src.gitMigration.repoCleaning import remove_lfs, split_large_files, remove_lfs_from_gitattributes

"""
This script migrates each repo specified in the architecture.yaml into a form that can either be uploaded to GitHub or the monorepo.
"""


architecture = Architecture.load_architecture("architecture.yaml")

REMOVE_LFS = True
REMOVE_LARGE_FILES = True
SPLIT_LARGE_FILES = False
numberIterations = 0
for repoID in architecture.repoIDs:
    numberIterations += len(architecture.get_repo_by_ID(repoID).get_branches_to_be_migrated())


with tqdm(total=numberIterations, desc="Migrating repository branches", unit="branches") as progress:
    for repoID in architecture.repoIDs:
        repo = architecture.get_repo_by_ID(repoID)
        git_repo = git.Repo(repo.path)

        if REMOVE_LFS:
            # Check if the repo uses LFS

            lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=repo.path,
                                       capture_output=True,
                                       text=True)
            # If yes remove LFS
            if lfs_check.stdout:
                remove_lfs(repo.path)

        for branch in repo.get_branches_to_be_migrated():
            git_repo.git.checkout(branch)
            if SPLIT_LARGE_FILES:
                split_large_files(repo.path)

            if REMOVE_LFS:
                remove_lfs_from_gitattributes(repo.path)
            progress.update(1)
        if "master" in git_repo.branches:
            git_repo.git.checkout("master")
        elif "main" in git_repo.branches:
            git_repo.git.checkout("main")

if REMOVE_LARGE_FILES:
    for repoID in tqdm(architecture.repoIDs, desc="Cleaning large files, this may take a while", unit="repos"):
        repo_path = architecture.get_repo_by_ID(repoID).path
        run_git_filter_repo(repo_path)