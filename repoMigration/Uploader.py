import logging
from abc import ABC
import yaml
import git

logger = logging.getLogger(__name__)


class Uploader(ABC):
    def __init__(self):
        data = yaml.safe_load(open("architecture.yaml"))

        self.namespaces = {}
        self.repoNames = {}
        self.branchesToBeMigrated = {}
        for repoID in data.keys():
            if data[repoID]["Branches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["StaleBranches"]
            elif data[repoID]["StaleBranches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["Branches"]
            else:
                self.branchesToBeMigrated[str(repoID)] = list(
                    set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))
            self.repoNames[str(repoID)] = data[repoID]["Name"]
            self.namespaces[str(repoID)] = data[repoID]["Namespace"]
        self.repoIDS = data.keys()

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
        repo = git.Repo.init("./repos/" + repo_name)
        # Create an empty README file
        readme_path = "./repos/" + repo_name + "/README.md"
        with open(readme_path, 'w') as f:
            f.write("# " + repo_name)
        # Add the README file to the repository
        repo.git.add("README.md")
        # Commit the changes
        repo.index.commit("Initial commit")
        return repo

    def add_subtree(self, name, subtree_repo_name, prefix=""):
        """
        Add a subtree to the repository.
        :param name: Repository name
        :param subtree_repo_name: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        branch = "master"  # Default branch is master
        repo = git.Repo("./repos/" + name)
        # Check if the subtree already exists
        subtree_path = prefix + "/" + subtree_repo_name
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            print(f"Subtree '{subtree_repo_name}' already exists at '{subtree_path}'.")
            return
        # Creates new branch in which the subtree is added
        """
        repo.create_head(subtreeRepoName)
        repo.heads[subtreeRepoName].checkout()
        """
        if subtree_repo_name in repo.remotes:
            repo.delete_remote(subtree_repo_name)
        repo.create_remote(subtree_repo_name, "../" + subtree_repo_name)
        repo.git.fetch(subtree_repo_name, branch)
        repo.git.subtree("add", "--prefix", prefix + "/" + subtree_repo_name, subtree_repo_name, branch)
        # Merges extra branch for subtree with master
        """
        repo.heads["master"].checkout()
        repo.git.merge("master", subtreeRepoName)
        repo.git.branch("-d", subtreeRepoName)
        """

    def add_subtree_branch(self, name, subtree_repo_name, branch, prefix=""):
        """
        Adds (multiple) branches of a repo as subtree to the repository.
        :param : Repository name
        :param subtree_repo_name: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        repo = git.Repo("./repos/" + name)
        # Check if the subtree already exists
        subtree_path = prefix + "/" + subtree_repo_name + "/" + branch
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            print(f"Subtree '{subtree_repo_name}' already exists at '{subtree_path}'.")
            return
        if subtree_repo_name in repo.remotes:
            repo.delete_remote(subtree_repo_name)
        repo.create_remote(subtree_repo_name, "../" + subtree_repo_name)
        repo.git.fetch(subtree_repo_name, branch)
        repo.git.subtree("add", "--prefix", subtree_path, subtree_repo_name, branch)

    def add_repos_as_subtree(self, target_repo_name, subtree_repo_IDS):
        """
        Adds only master branch as subtree to the target repository.
        :param target_repo_name: Name of the target GitHub repository
        :param subtree_repo_IDS: IDs of the repositories to be uploaded as subtrees
        """
        target_repo = self.init_repo(target_repo_name)
        for repoID in subtree_repo_IDS:
            repo_name = self.repoNames[repoID]
            logger.info(f"Uploading {repo_name} as a subtree to {target_repo_name}...")
            namespace = self.namespaces[repoID]
            multiple_branches = len(self.branchesToBeMigrated[repoID]) > 1
            for branch in self.branchesToBeMigrated[repoID]:
                if multiple_branches:
                    self.add_subtree_branch(target_repo_name, repo_name, prefix=namespace, branch=branch)
                    logger.info(f"Branch {branch} uploaded as a subtree.")
                else:
                    self.add_subtree(target_repo_name, repo_name, prefix=namespace)
            logger.info(f"Repository {repo_name} uploaded as a subtree to {target_repo_name}.")
