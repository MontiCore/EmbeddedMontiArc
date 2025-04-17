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