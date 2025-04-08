import os
import subprocess

import gitlab
import yaml
from git import RemoteProgress
from tqdm import tqdm
import git

from sourceAnalysis.Git import Git


class Gitlab(Git):
    def __init__(self, sourceURL: str, privateToken, repoIDS):
        self.__privateToken = privateToken
        self.repoIDS = repoIDS

        self.gl = gitlab.Gitlab(url=sourceURL, private_token=self.__privateToken)
        self.gl.auth()

    def scanRepos(self):
        architecture = {}
        for repoID in tqdm(self.repoIDS, desc="Scanning repositories"):
            repoData= {}
            repo = self.getRepo(repoID)
            repoData['Name'] = self.getRepoName(repo)
            repoData['Branches'] = self.getBranches(repo)
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

    def getDockerImages(self,repo):
        data = []
        try:
            images =repo.repositories.list(all=True)
            for image in images:
                data.append(image.name)
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
        if lfs_check.stdout:
            print("LFS-Objekte gefunden, LFS-Objekte werden heruntergeladen...")
            process = subprocess.Popen(["git", "lfs", "pull"], cwd=clone_path, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            for line in process.stdout:
                print(line.strip())
            rc = process.poll()
        else:
            print("LFS-Objekte nicht gefunden, keine weiteren Schritte erforderlich.")
        #self.removeRemoteOrigin(clone_path)

    def removeRemoteOrigin(self, repo_path):
        try:
            repo = git.Repo(repo_path)
            origin = repo.remote(name='origin')
            repo.delete_remote(origin)
            print("Remote 'origin' wurde erfolgreich entfernt.")
        except Exception as e:
            print(f"Fehler beim Entfernen des Remote 'origin': {e}")

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