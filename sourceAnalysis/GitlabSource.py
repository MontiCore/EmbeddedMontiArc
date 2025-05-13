import datetime
import os
import subprocess


import gitlab
import yaml
from git import RemoteProgress
from tqdm import tqdm
import git

from sourceAnalysis.Git import Git
from sourceAnalysis.mavenSecrets import find_env_vars_in_repo


class Gitlab(Git):
    def __init__(self, sourceURL: str, privateToken, repoIDS):
        self.__privateToken = privateToken
        self.repoIDS = repoIDS

        self.gl = gitlab.Gitlab(url=sourceURL, private_token=self.__privateToken)
        self.gl.auth()
    #ToDo: Maybe list existing secrets
    def scanRepos(self):
        architecture = {}
        for repoID in tqdm(self.repoIDS, desc="Scanning repositories"):
            repoData= {}
            repo = self.getRepo(repoID)
            repoData['Name'] = self.getRepoName(repo)
            repoData['Namespace'] = self.getNamespace(repo)
            branches = self.getBranches(repo)
            staleBranches = self.getStaleBranches(repo)
            repoData['Branches'] = []
            repoData['StaleBranches'] = []
            repoData['Secrets'] = {}
            envVariables = find_env_vars_in_repo("./repos/" + repoData['Name'])
            repoData['Secrets']["GITLABTOKEN"] = {"Value": self.__privateToken, "Secret": "Y"}
            repoData['Secrets']["CI_API_V4_URL"] = {"Value": self.gl.api_url, "Secret": "N"}
            if envVariables:
                for secret in envVariables:
                    if secret == "CI_JOB_TOKEN" or secret == "CI_API_V4_URL" or secret == "CI_PROJECT_ID":
                        continue
                    else:
                        repoData['Secrets'][secret]= {"Value":"Please add a value", "Secret":"Y, if it should be saved as a secret. E, if it already exists. The value should then be the name of the according secret. Default is N"}
            for b in branches:
                if b not in staleBranches:
                    repoData['Branches'].append(b)
                else:
                    repoData['StaleBranches'].append(b)
            if repoData['Branches'] == []:
                repoData['Branches'] = None
            if repoData['StaleBranches'] == []:
                repoData['StaleBranches'] = None

            dockerImages = self.getDockerImages(repo)
            if dockerImages:
                repoData['DockerImages'] = dockerImages
            mavenArtifacts = self.getMavenArtifacts(repo)
            if mavenArtifacts:
                repoData['MavenArtifacts'] = mavenArtifacts
            architecture[repoID] = repoData


        yaml.dump(architecture, open("architecture.yaml", 'w'))

    def getRepoName(self,repo):
        """
        Get the name of the repository.
        :param repo: Repository object
        :return: str - Name of the repository
        """
        return repo.name

    def cloneRepos(self):
        """
        Clone repositories from GitLab to the local machine.
        """
        print("Cloning Repositories...")
        for repo in self.repoIDS:
            print("-------------------------------")
            self.cloneRepo(repo, "./repos/")
            self.removeRemoteOrigin("./repos/" + self.getRepoName(self.getRepo(repo)))

    def getRepo(self,repoID : str):
        """
        Get the repository object from GitLab by its id.
        :param repoID: GitLab repository ID
        :return: Repository object
        """
        repo = self.gl.projects.get(repoID)
        return repo

    def getBranches(self, repo):
        data = []
        branches =repo.branches.list(all=True)
        for branch in branches:
            data.append(branch.name)
        return data

    def getStaleBranches(self, repo):
        cutoff_date = datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(days=90)
        stale_branches = []
        branches = repo.branches.list(all=True)
        for branch in branches:
            commit_date = datetime.datetime.fromisoformat(branch.commit['committed_date']).replace(tzinfo=datetime.timezone.utc)
            if commit_date < cutoff_date:
                stale_branches.append(branch.name)
        return stale_branches

    def getDockerImages(self,repo):
        #data = []
        #try:
        #    images =repo.repositories.list(all=True)
        #    for image in images:
        #        data.append(image.name)
        #except gitlab.exceptions.GitlabListError:
        #    pass
        #return data
        data = []
        try:
            images = repo.repositories.list(all=True)
            for image in images:
                #image_data = {"name": image.name, "tags": []}
                try:
                    tags = image.tags.list(all=True)
                    for tag in tags:
                        if image.name:
                            data.append(image.name+":" + tag.name)
                        else:
                            data.append(self.getRepoName(repo).lower()+":"+tag.name)
                        #image_data["tags"].append(tag.name)
                except gitlab.exceptions.GitlabListError:
                    pass
                #data.append(image_data)
        except gitlab.exceptions.GitlabListError:
            pass
        return data

    def getMavenArtifacts(self, repo):
        data = []
        try:
            artifacts =repo.packages.list(all=True)
            for artifact in artifacts:
                data.append(artifact.name)
        except gitlab.exceptions.GitlabListError:
            pass
        return data

    def getNamespace(self, repo):
        return repo.namespace['full_path']

    def cloneRepo(self, repo_id, clone_path):
        repo = self.getRepo(repo_id)
        name = self.getRepoName(repo)
        clone_path = os.path.join(clone_path, name)
        repo_url = repo.http_url_to_repo.replace("https://", f"https://oauth2:{self.__privateToken}@")
        print(f"Cloning {repo_id} to {clone_path}")
        if not os.path.exists(clone_path):
            os.makedirs(clone_path)
        else:
            print(f"Directory {clone_path} already exists, skipping clone.")
            return

        git.Repo.clone_from(repo_url, clone_path, branch='master', progress=CloneProgress())

        print(f"Cloning {repo_id} finished")
        lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=clone_path, capture_output=True, text=True)
        if lfs_check.stdout and True: #Todo: Activate
            print("LFS-Objekte gefunden, LFS-Objekte werden heruntergeladen...")
            process = subprocess.Popen(["git", "lfs", "pull"], cwd=clone_path, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            for line in process.stdout:
                print(line.strip())
            rc = process.poll()
        else:
            print("LFS-Objekte nicht gefunden, keine weiteren Schritte erforderlich.")
        self.chekoutBranches(name)
        #self.removeRemoteOrigin(clone_path)

    def chekoutBranches(self, repoName):
        """
        Checkout all branches available in the remote repository.
        :param repoName: Name of the repository to be checked out
        """
        repo = git.Repo("./repos/" + repoName)
        print(f"Fetching all branches for {repoName}...")
        repo.remotes.origin.fetch()  # Fetch all branches from the remote
        print(f"Checking out branches for {repoName}...")
        for branch in repo.remotes.origin.refs:  # Iterate over all remote branches
            branch_name = branch.name.split('/')[-1]  # Extract branch name
            if branch_name == "HEAD":
                continue
            try:
                repo.git.checkout('-B', branch_name, branch.name)  # Create and checkout local branch tracking the remote
                print(f"Checked out branch {branch_name}.")
            except Exception as e:
                print(f"Error checking out branch {branch_name}: {e}")
            repo.git.checkout('master')  # Checkout the master branch

    def removeRemoteOrigin(self, repo_path):
        try:
            repo = git.Repo(repo_path)
            origin = repo.remote(name='origin')
            repo.delete_remote(origin)
            print("Remote 'origin' wurde erfolgreich entfernt.")
        except Exception as e:
            pass

class CloneProgress(RemoteProgress):
    def __init__(self):
        super().__init__()
        self.pbar = tqdm()

    def update(self, op_code, cur_count, max_count=None, message=''):
        self.pbar.total = max_count
        self.pbar.n = cur_count
        self.pbar.refresh()



if __name__ == '__main__':
    dr = Gitlab("repos.yaml")
    dr.scanRepos()
    dr.cloneRepos()