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
                self.branchesToBeMigrated[str(repoID)] = list(set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))
            self.repoNames[str(repoID)] = data[repoID]["Name"]
            self.namespaces[str(repoID)] = data[repoID]["Namespace"]
        self.repoIDS = data.keys()


    def delete_local_branch(self, repoPath, branch_name):
        try:
            repo = git.Repo(repoPath)
            repo.git.branch('-D', branch_name)
            print(f"Branch '{branch_name}' was deleted")
        except Exception as e:
            print(f"Fehler beim Löschen des Branches '{branch_name}': {e}")

    def initRepo(self, repoName):
        """
        Initialize a git repository in the given path and makes init commit.
        :param repoName: Name of the repository
        """
        repo = git.Repo.init("./repos/"+ repoName)
        # Create an empty README file
        readme_path = "./repos/" + repoName + "/README.md"
        with open(readme_path, 'w') as f:
            f.write("# " + repoName)
        # Add the README file to the repository
        repo.git.add("README.md")
        # Commit the changes
        repo.index.commit("Initial commit")
        return repo

    def addSubtree(self,repoName, subtreeRepoName ,prefix = ""):
        """
        Add a subtree to the repository.
        :param repoName: Repository name
        :param subtreeRepoName: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        branch = "master" #Default branch is master
        repo = git.Repo("./repos/" + repoName)
        # Check if the subtree already exists
        subtree_path = prefix + "/" + subtreeRepoName
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            print(f"Subtree '{subtreeRepoName}' already exists at '{subtree_path}'.")
            return
        #repo.remotes.add(subtreeRepoName, "./repos/" + subtreeRepoName)
        #repo.remotes[0].set_url(subtreeRepoName, "./repos/" + subtreeRepoName)
        """
        repo.create_head(subtreeRepoName)
        repo.heads[subtreeRepoName].checkout()
        """
        if subtreeRepoName in repo.remotes:
            repo.delete_remote(subtreeRepoName)
        repo.create_remote(subtreeRepoName, "../" + subtreeRepoName)
        repo.git.fetch(subtreeRepoName, branch)
        repo.git.subtree("add", "--prefix", prefix +"/" + subtreeRepoName, subtreeRepoName, branch)
        """
        repo.heads["master"].checkout()
        repo.git.merge("master", subtreeRepoName)
        repo.git.branch("-d", subtreeRepoName)
        """
        #repo.git.subtree("merge", "--prefix", prefix +"/" + subtreeRepoName, subtreeRepoName, branch)

    def addSubtreeBranch(self,repoName, subtreeRepoName , branch, prefix = ""):
        """
        Adds (multiple) branches of a repo as subtree to the repository.
        :param repoName: Repository name
        :param subtreeRepoName: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        repo = git.Repo("./repos/" + repoName)
        # Check if the subtree already exists
        subtree_path = prefix + "/" + subtreeRepoName + "/" + branch
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            print(f"Subtree '{subtreeRepoName}' already exists at '{subtree_path}'.")
            return
        if subtreeRepoName in repo.remotes:
            repo.delete_remote(subtreeRepoName)
        repo.create_remote(subtreeRepoName, "../" + subtreeRepoName)
        repo.git.fetch(subtreeRepoName, branch)
        repo.git.subtree("add", "--prefix", subtree_path, subtreeRepoName, branch)


    def addReposAsSubtree(self, targetRepoName, subtreeRepoIDs):
        """
        Adds only master branch as subtree to the target repository.
        :param targetRepoName: Name of the target GitHub repository
        :param subtreeRepoIDs: IDs of the repositories to be uploaded as subtrees
        """
        targetRepo = self.initRepo(targetRepoName)
        for repoID in subtreeRepoIDs:
            repoName = self.repoNames[repoID]
            logger.info(f"Uploading {repoName} as a subtree to {targetRepoName}...")
            namespace = self.namespaces[repoID]
            multipleBranches = len(self.branchesToBeMigrated[repoID]) > 1
            for branch in self.branchesToBeMigrated[repoID]:
                if multipleBranches:
                    self.addSubtreeBranch(targetRepoName, repoName, prefix=namespace, branch=branch)
                    logger.info(f"Branch {branch} uploaded as a subtree.")
                else:
                    self.addSubtree(targetRepoName, repoName, prefix=namespace)
            logger.info(f"Repository {repoName} uploaded as a subtree to {targetRepoName}.")