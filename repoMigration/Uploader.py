from abc import ABC
import yaml
import git

class Uploader(ABC):
    def __init__(self):
        data = yaml.safe_load(open("architecture.yaml"))

        self.branchesToBeMigrated = {}
        for repoID in data.keys():
            if data[repoID]["Branches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["StaleBranches"]
            elif data[repoID]["StaleBranches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["Branches"]
            else:
                self.branchesToBeMigrated[str(repoID)] = list(set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))
        print(self.branchesToBeMigrated)

        self.repoIDS = data.keys()
        self.repoNames = {}

        for repoID in self.repoIDS:
            self.repoNames[str(repoID)] = data[repoID]["Name"]
        print(self.repoNames)


    def delete_local_branch(slef, repoPath, branch_name):
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

    def addSubtree(self,repoName, subtreeRepoName ,prefix = "" , branch = "master"):
        """
        Add a subtree to the repository.
        :param repoName: Repository name
        :param subtreeRepoName: Name of the repository to be added as a subtree
        :param branch: Branch of the subtree to be added
        """
        repo = git.Repo("./repos/" + repoName)
        # Check if the subtree already exists
        subtree_path = prefix + "/" + subtreeRepoName
        if subtree_path in [item.path for item in repo.tree().traverse()]:
            print(f"Subtree '{subtreeRepoName}' already exists at '{subtree_path}'.")
            return
        #repo.remotes.add(subtreeRepoName, "./repos/" + subtreeRepoName)
        #repo.remotes[0].set_url(subtreeRepoName, "./repos/" + subtreeRepoName)
        if subtreeRepoName in repo.remotes:
            repo.delete_remote(subtreeRepoName)
        repo.create_remote(subtreeRepoName, "../" + subtreeRepoName)
        repo.git.fetch(subtreeRepoName)
        repo.git.subtree("add", "--prefix", prefix +"/" + subtreeRepoName, subtreeRepoName, branch)