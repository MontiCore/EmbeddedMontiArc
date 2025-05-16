import logging
from abc import ABC

import git
import yaml
import os

logger = logging.getLogger(__name__)

class Git(ABC):
    def __init__(self):
        pass

    def reset_remote_origin(self, repo, new_remote_url):
        """
        Removes the remote origin and replaces it with a new one.
        :param repo: Repo object
        :param new_remote_url: New remote URL
        """
        try:
            # Remove remote origin if it exists
            self.remove_remote_origin(repo)

            # Add new remote origin
            repo.create_remote('origin', new_remote_url)
            logger.info(f"Remote 'origin' wurde erfolgreich auf '{new_remote_url}' gesetzt.")
        except Exception as e:
            logger.error(f"Fehler beim Zurücksetzen des Remote 'origin': {e}")

    def remove_remote_origin(self, repo):
        """
        Remove the remote origin from the repository.
        :param repo: Repo Object
        :return:
        """
        try:
            if 'origin' in repo.remotes:
                origin = repo.remote(name='origin')
                repo.delete_remote(origin)
                logging.info("Remote 'origin' wurde erfolgreich entfernt.")
        except Exception as e:
            logger.error(f"Fehler beim Zurücksetzen des Remote 'origin': {e}")

    def set_upstream_for_branches(self, repo, remote_name="origin"):
        """
        Adds upstream branches to all local branches in the given repository.
        :param repo: Repo object
        :param remote_name: Name of remote origin. Default is origin
        """
        try:
            for branch in repo.branches:
                if branch.tracking_branch() is None:  # Check if the branch has an upstream branch
                    remote_branch = f"{remote_name}/{branch.name}"
                    repo.git.branch("--set-upstream-to", remote_branch, branch.name)
                    print(f"Upstream-Branch für '{branch.name}' auf '{remote_branch}' gesetzt.")
                else:
                    print(f"Upstream-Branch für '{branch.name}' ist bereits gesetzt.")
        except Exception as e:
            print(f"Fehler beim Setzen der Upstream-Branches: {e}")

    def delete_local_branch(self, repoPath, branch_name):
        """
        Deletes a local branch in the given repository.
        :param repoPath: Path to the repository
        :param branch_name: Name of the branch to be deleted
        """
        try:
            repo = git.Repo(repoPath)
            repo.git.branch('-D', branch_name)
            print(f"Branch '{branch_name}' was deleted")
        except Exception as e:
            print(f"Fehler beim Löschen des Branches '{branch_name}': {e}")

    def init_repo(self, repo_name):
        """
        Initialize a git repository in the given path and makes init commit.
        :param repo_name: Name of the repository
        """
        repo = git.Repo.init(os.path.join(os.getcwd(), "repos", repo_name))
        # Create an empty README file
        readme_path = "./repos/" + repo_name + "/README.md"
        with open(readme_path, 'w') as f:
            f.write("# " + repo_name)
        # Add the README file to the repository
        repo.git.add("README.md")
        # Commit the changes
        repo.index.commit("Initial commit")
        if "master" not in repo.branches:
            repo.create_head("master")
            repo.heads["master"].checkout()
        return repo

    def add_subtree(self, repo, subtree_repo_name, prefix="", branch="master"):
        """
        Add a subtree to the repository.
        :param name: Repository name
        :param subtree_repo_name: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        # Check if the subtree already exists
        subtree_path = os.path.join(prefix, subtree_repo_name)
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            logger.warning(f"Subtree '{subtree_repo_name}' already exists at '{subtree_path}'.")
            return
        # Creates new branch in which the subtree is added
        if subtree_repo_name in repo.remotes:
            repo.delete_remote(subtree_repo_name)
        repo.create_remote(subtree_repo_name, os.path.join("../..", subtree_repo_name))
        repo.git.fetch(subtree_repo_name, branch)
        repo.git.subtree("add", "--prefix", prefix + "/" + subtree_repo_name, subtree_repo_name, branch)
        logger.info(f"Branch {branch} uploaded as a subtree.")


    def add_subtree_branch(self, repo, subtree_repo_name, branch, prefix=""):
        """
        Adds (multiple) branches of a repo as subtree to the repository.
        :param name: Repository name
        :param subtree_repo_name: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        :param prefix: Prefix for the subtree path
        """

        # Check if the subtree already exists
        subtree_path = os.path.join(prefix, subtree_repo_name, branch)
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            logger.info(f"Subtree '{branch}' already exists at '{subtree_path}'.")
            return
        if subtree_repo_name in repo.remotes:
            repo.delete_remote(subtree_repo_name)
        repo.create_remote(subtree_repo_name, os.path.join("../..", subtree_repo_name))
        repo.git.fetch(subtree_repo_name, branch)
        repo.git.subtree("add", "--prefix", subtree_path, subtree_repo_name, branch)

    def add_repos_as_subtree(self, target_repo_name, target_repo_namespace, subtree_repo_IDS):
        """
        Adds only master branch as subtree to the target repository.
        :param target_repo_name: Name of the target GitHub repository
        :param subtree_repo_IDS: IDs of the repositories to be uploaded as subtrees
        """
        target_repo = self.init_repo(target_repo_name)
        branch_name = "Migration_" + str(datetime.now().strftime("%Y-%m-%d_at_%H-%M-%S"))
        if branch_name in target_repo.branches:
            user_input = input((f"Branch {branch_name} already exists. Do you want to delete it? (y/n)"))
            if user_input.lower() == "y":
                target_repo.delete_head(branch_name)
        target_repo.create_head(branch_name)
        # target_repo.git.symbolic_ref(f"refs/heads/{branch_name}", "refs/heads/empty")  # ToDO: Test
        target_repo.heads[branch_name].checkout()
        for repoID in subtree_repo_IDS:
            repo_name = self.repoNames[repoID]
            repoNamespace = "/".join([i for i in self.namespaces[repoID].split("/") if i not in target_repo_namespace])
            multiple_branches = len(self.branchesToBeMigrated[repoID]) > 1
            logger.info(f"Uploading {repo_name} as a subtree to {repoNamespace}...")
            for branch in self.branchesToBeMigrated[repoID]:
                if multiple_branches:
                    self.add_subtree_branch(target_repo, repo_name, prefix=repoNamespace, branch=branch)
                else:
                    self.add_subtree(target_repo, repo_name, prefix=repoNamespace, branch=branch)