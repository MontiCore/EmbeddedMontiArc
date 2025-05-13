import datetime
import os
import subprocess

import gitlab
import yaml
from git import RemoteProgress
from pygments.lexer import inherit
from tqdm import tqdm
import git

from sourceAnalysis.Git import Git
from sourceAnalysis.mavenSecrets import find_env_vars_in_repo


class Gitlab(Git):
    def __init__(self, source_url: str, private_token, repoIDS):
        self.__privateToken = private_token
        self.repoIDS = repoIDS

        self.gl = gitlab.Gitlab(url=source_url, private_token=self.__privateToken)
        self.gl.auth()

    def scan(self):
        """
        Scans all repositories and create a yaml file with the architecture of the repositories.
        """
        architecture = {}
        for repoID in tqdm(self.repoIDS, desc="Scanning repositories"):
            repo_data = {}
            repo = self.getRepo(repoID)
            repo_data['Name'] = self.getRepoName(repo)
            repo_data['Namespace'] = self.getNamespace(repo)
            branches = self.getBranches(repo)
            stale_branches = self.getStaleBranches(repo)
            repo_data['Branches'] = []
            repo_data['StaleBranches'] = []
            repo_data['Secrets'] = {}
            # Get env variables used by maven. Should be recreated in the new repo
            env_variables = find_env_vars_in_repo("./repos/" + repo_data['Name'])
            repo_data['Secrets']["GITLABTOKEN"] = {"Value": self.__privateToken, "Secret": "Y"}
            repo_data['Secrets']["CI_API_V4_URL"] = {"Value": self.gl.api_url, "Secret": "N"}
            if env_variables:
                for secret in env_variables:
                    if secret == "CI_JOB_TOKEN" or secret == "CI_API_V4_URL" or secret == "CI_PROJECT_ID":
                        continue
                    else:
                        repo_data['Secrets'][secret] = {"Value": "Please add a value",
                                                        "Secret": "Y, if it should be saved as a secret. E, if it already exists. The value should then be the name of the according secret. Default is N"}
            for b in branches:
                if b not in stale_branches:
                    repo_data['Branches'].append(b)
                else:
                    repo_data['StaleBranches'].append(b)
            if not repo_data['Branches']:
                repo_data['Branches'] = None
            if not repo_data['StaleBranches']:
                repo_data['StaleBranches'] = None

            docker_images = self.getDockerImages(repo)
            if docker_images:
                repo_data['DockerImages'] = docker_images
            maven_artifacts = self.getMavenArtifacts(repo)
            if maven_artifacts and False:
                repo_data['MavenArtifacts'] = maven_artifacts
            architecture[repoID] = repo_data

        yaml.dump(architecture, open("architecture.yaml", 'w'))

    def getRepoName(self, repo):
        """
        Get the name of the repository.
        :param repo: Repository object
        :return: str - Name of the repository
        """
        return repo.name

    def clone(self):
        """
        Clone repositories from GitLab to the local machine.
        """
        print("Cloning Repositories...")
        for repo in self.repoIDS:
            print("-------------------------------")
            self.clone_repo(repo, "./repos/")
            self.remove_remote_origin("./repos/" + self.getRepoName(self.getRepo(repo)))

    def getRepo(self, repoID: str):
        """
        Get the repository object from GitLab by its id.
        :param repoID: GitLab repository ID
        :return: Repository object
        """
        repo = self.gl.projects.get(repoID)
        return repo

    def getBranches(self, repo):
        """
        Get all branches from the repository.
        :param repo: GitLab Repo object
        :return: list - List of branches
        """
        data = []
        branches = repo.branches.list(all=True)
        for branch in branches:
            data.append(branch.name)
        return data

    def getStaleBranches(self, repo):
        """
        Get all branches that are older than 90 days.
        :param repo: GitLab  Repo object
        :return: list - List of stale branches
        """
        cutoff_date = datetime.datetime.now(datetime.timezone.utc) - datetime.timedelta(days=90)
        stale_branches = []
        branches = repo.branches.list(all=True)
        for branch in branches:
            commit_date = datetime.datetime.fromisoformat(branch.commit['committed_date']).replace(
                tzinfo=datetime.timezone.utc)
            if commit_date < cutoff_date:
                stale_branches.append(branch.name)
        return stale_branches

    def getDockerImages(self, repo):
        """
        Get all docker images from the repository.
        :param repo: GitLab Repo object
        :return: list - List of docker images
        """
        data = []
        try:
            images = repo.repositories.list(all=True)
            for image in images:
                # image_data = {"name": image.name, "tags": []}
                try:
                    tags = image.tags.list(all=True)
                    for tag in tags:
                        if image.name:
                            data.append(image.name + ":" + tag.name)
                        else:
                            data.append(self.getRepoName(repo).lower() + ":" + tag.name)
                        # image_data["tags"].append(tag.name)
                except gitlab.exceptions.GitlabListError:
                    pass
                # data.append(image_data)
        except gitlab.exceptions.GitlabListError:
            pass
        return data

    def getMavenArtifacts(self, repo):
        """
        Get all maven artifacts from the repository.
        :param repo: GitLab Repo object
        :return: list - List of maven artifacts
        """
        data = []
        try:
            artifacts = repo.packages.list(all=True)
            for artifact in artifacts:
                data.append(artifact.name)
        except gitlab.exceptions.GitlabListError:
            pass
        return data

    def getNamespace(self, repo):
        """
        Get the namespace of the repository.
        :param repo: GitLab Repo object
        :return: Str - Namespace of the repository
        """
        return repo.namespace['full_path']

    def clone_repo(self, repo_id, clone_path):
        """
        Clone a repository from GitLab to the local machine. Additionally it checks out all branches and removes the remote origin.
        :param repo_id: Repository ID in GitLab
        :param clone_path: Path to clone the repository to
        """
        repo = self.getRepo(repo_id)
        name = self.getRepoName(repo)
        clone_path = os.path.join(clone_path, name)
        # Create authorized URL with private token for cloning
        repo_url = repo.http_url_to_repo.replace("https://", f"https://oauth2:{self.__privateToken}@")
        print(f"Cloning {repo_id} to {clone_path}")
        # If a folder with the repos name already exists, skip cloning
        if not os.path.exists(clone_path):
            os.makedirs(clone_path)
        else:
            print(f"Directory {clone_path} already exists, skipping clone.")
            return
        # Clone
        git.Repo.clone_from(repo_url, clone_path, branch='master', progress=CloneProgress())
        print(f"Cloning {repo_id} finished")

        # Check if LFS is used and download LFS objects if necessary
        lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=clone_path, capture_output=True, text=True)
        if lfs_check.stdout:
            print("LFS-Objekte gefunden, LFS-Objekte werden heruntergeladen...")
            process = subprocess.Popen(["git", "lfs", "pull"], cwd=clone_path, stdout=subprocess.PIPE,
                                       stderr=subprocess.PIPE, text=True)
            for line in process.stdout:
                print(line.strip())
            rc = process.poll()

        self.chekout_branches(name)
        self.remove_remote_origin(clone_path)

    def chekout_branches(self, repoName):
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
                repo.git.checkout('-B', branch_name,
                                  branch.name)  # Create and checkout local branch tracking the remote
                print(f"Checked out branch {branch_name}.")
            except Exception as e:
                print(f"Error checking out branch {branch_name}: {e}")
            repo.git.checkout('master')  # Checkout the master branch

    def remove_remote_origin(self, repo_path):
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
    dr.scan()
    dr.clone()
