import os
import subprocess
import gitlab
import yaml

class GitlabDriver:
    def __init__(self, path):
        data = yaml.safe_load(open(path))
        self.gitlabURL = data['URL']
        self.__privateToken = data['PrivateToken']
        repoIDS = data['Repos']

        self.gl = gitlab.Gitlab(url=self.gitlabURL, private_token=self.__privateToken)
        self.gl.auth()

        architecture = {}

        for repo in repoIDS:
            repoData= {}
            self.repo = self.getRepo(repo)
            repoData['Name'] = self.repo.name
            repoData['Branches'] = self.getBranches(self.repo)
            repoData['DockerImages'] = self.getDockerImages(self.repo)
            repoData['MavenArtifacts'] = self.getMavenArtifacts(self.repo)
            architecture[repo] = repoData
        yaml.dump(architecture, open("architecture.yaml", 'w'))
        for repo in repoIDS:
            self.cloneRepo(repo,"/repos/"+architecture[repo]['Name'])

    def getRepo(self,repoID):
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
        images =repo.repositories.list(all=True)
        for image in images:
            data.append(image.name)
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
        repo_url = repo.http_url_to_repo.replace("https://", f"https://oauth2:{self.__privateToken}@")
        print(f"Cloning {repo_id} to {clone_path}")
        if not os.path.exists(clone_path):
            os.makedirs(clone_path)
        subprocess.run(["git", "clone", repo_url, clone_path], check=True)
        print(f"Repository {repo.name} wurde in {clone_path} geklont.")

        lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=clone_path, capture_output=True, text=True)
        if lfs_check.stdout:
            print("LFS-Objekte gefunden, LFS-Objekte werden heruntergeladen...")
            subprocess.run(["git", "lfs", "pull"], cwd=clone_path, check=True)

if __name__ == '__main__':
    dr = GitlabDriver("repos.yaml")