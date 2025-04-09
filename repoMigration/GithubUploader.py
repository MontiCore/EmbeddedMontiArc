import logging

from git import RemoteProgress
from tqdm import tqdm

from repoMigration.Uploader import Uploader
from github import Github, GithubException
from github import Auth
import git

# Logger konfigurieren
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

class GithubUploader(Uploader):
    def __init__(self, privateToken, sourceURL: str = "https://api.github.com/"):
        super().__init__()
        self.__privateToken = privateToken
        auth = Auth.Token(self.__privateToken)
        self.g = Github(auth=auth, base_url=sourceURL)

    def createPrivateRepo(self, repoName):
        """
        Create a private repository on the target Git instance.
        :param repoName: Name of the repository to be created
        :return: Repository object
        """
        try:
            repo = self.g.get_user().create_repo(repoName, private=True)
        except GithubException as e:
            if e.status == 422:
                logger.info(f"Repository '{repoName}' already exists.")
                repo = self.g.get_user().get_repo(repoName)
                if input("Delete existing repository? (y/n): ").lower() == 'y':
                    repo.delete()
                    logger.info(f"Repository '{repoName}' deleted.")
                    repo = self.g.get_user().create_repo(repoName, private=True)
                    logger.info(f"New repository '{repoName}' created.")
                else:
                    logger.info(f"Keeping existing repository '{repoName}'.")
                    repo = self.g.get_user().get_repo(repoName)
            else:
                raise
        return repo

    def createPublicRepo(self, repoName):
        """
        Create a public repository on the target Git instance.
        :param repoName: Name of the repository to be created
        :return: Repository object
        """
        repo = self.g.get_user().create_repo(repoName, private=False)
        return repo

    def listPrivateRepos(self):
        """
        List all private repositories on the target Git instance.
        :return: List of repository names
        """
        repos = self.g.get_user().get_repos()
        return [repo.name for repo in repos if repo.private]

    def uploadRepo(self, repoID):
        """
        Upload a repository to the target Git instance.
        :param repoID: RepositoryID to be uploaded
        """
        repo = git.Repo("./repos/" + self.repoNames[repoID])
        logger.info(f"Uploading {self.repoNames[repoID]} to the target Git instance...")

        # Create a new private repository on the target Git instance
        newRepo = self.createPrivateRepo(self.repoNames[repoID])
        existingBranches = [b.name for b in newRepo.get_branches()]
        remote_url = newRepo.clone_url.replace("https://", f"https://{self.__privateToken}@")
        self.reset_remote_origin(repo, remote_url)
        for branch in self.branchesToBeMigrated[repoID]:

            if branch in existingBranches:
                logger.info(f"Branch {branch} already exists in the target repository.")
            else:
                logger.info(f"Uploading {branch} branch...")
                with PushProgress() as progress:
                    repo.remote(name="origin").push(refspec=f"{branch}:{branch}", force=True, progress=progress)
                logger.info(f"Branch {branch} uploaded successfully.")
        newRepo.edit(default_branch="master")

    def reset_remote_origin(self, repo, new_remote_url):
        """
        Entfernt das Remote-Repository 'origin' und setzt es mit einem neuen Wert zurück.
        :param repo: Repo Objekt, das das Remote-Repository enthält
        :param new_remote_url: Neue URL für das Remote-Repository
        """
        try:
            # Entferne das Remote-Repository 'origin', falls vorhanden
            if 'origin' in repo.remotes:
                origin = repo.remote(name='origin')
                repo.delete_remote(origin)
                logger.info("Remote 'origin' wurde erfolgreich entfernt.")

            # Füge das neue Remote-Repository hinzu
            repo.create_remote('origin', new_remote_url)
            logger.info(f"Remote 'origin' wurde erfolgreich auf '{new_remote_url}' gesetzt.")
        except Exception as e:
            logger.error(f"Fehler beim Zurücksetzen des Remote 'origin': {e}")



class PushProgress(RemoteProgress):
    def __init__(self):
        super().__init__()
        self.pbar = tqdm(desc="Pushing", unit="objects")

    def update(self, op_code, cur_count, max_count=None, message=''):
        self.pbar.total = max_count
        self.pbar.n = cur_count
        self.pbar.refresh()

    def close(self):
        self.pbar.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()