import logging
import os

import requests
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
    def __init__(self, githubToken, gitlabToken, sourceURL: str = "https://api.github.com/"):
        super().__init__()
        self.__githubToken = githubToken
        self.__gitlabToken = gitlabToken
        auth = Auth.Token(self.__githubToken)
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
        try:
            repo = self.g.get_user().create_repo(repoName, private=False)
        except GithubException as e:
            if e.status == 422:
                logger.info(f"Repository '{repoName}' already exists.")
                repo = self.g.get_user().get_repo(repoName)
                if input("Delete existing repository? (y/n): ").lower() == 'y':
                    repo.delete()
                    logger.info(f"Repository '{repoName}' deleted.")
                    repo = self.g.get_user().create_repo(repoName, private=False)
                    logger.info(f"New repository '{repoName}' created.")
                else:
                    logger.info(f"Keeping existing repository '{repoName}'.")
                    repo = self.g.get_user().get_repo(repoName)
            else:
                raise
        return repo

    def getOrCreateRepo(self, repoName):
        """
        Get or create a repository on the target Git instance.
        :param repoName: Name of the repository to be created
        :return: Repository object
        """
        try:
            repo = self.g.get_user().get_repo(repoName)
            logger.info(f"Repository '{repoName}' already exists.")
        except GithubException as e:
            if e.status == 404:
                logger.info(f"Repository '{repoName}' does not exist.")
                visibility = input("Create public repository? (y/n): ").lower()
                if visibility == 'y':
                    repo = self.createPublicRepo(repoName)
                else:
                    repo = self.createPrivateRepo(repoName)
            else:
                raise
        return repo

    def createSecrets(self, github_repo, secrets):
        """
        Create secrets for the repository on the target Git instance.
        :param repoID:
        :param secrets:
        :return:
        """
        #ToDo: check if secrets already exist
        for name,secret in secrets.items():
            github_repo.create_secret(name, secret)

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
        print(repo.branches)
        #self.set_upstream_for_branches(repo)

        # Create a new private repository on the target Git instance
        #newRepo = self.createPrivateRepo(self.repoNames[repoID])
        newRepo = self.createPublicRepo(self.repoNames[repoID])
        secrets = {"GITLABTOKEN": self.__gitlabToken, "CI_API_V4_URL": "https://git.rwth-aachen.de/api/v4", "CI_PROJECT_ID": repoID}
        self.createSecrets(newRepo,secrets)

        existingBranches = [b.name for b in newRepo.get_branches()]
        remote_url = newRepo.clone_url.replace("https://", f"https://{self.__githubToken}@")
        self.reset_remote_origin(repo, remote_url)
        for branch in self.branchesToBeMigrated[repoID]:
            if branch in existingBranches:
                logger.info(f"Branch {branch} already exists in the target repository.")
            else:
                logger.info(f"Uploading {branch} branch...")
                with (PushProgress() as progress):
                    a = repo.remote(name="origin").push(refspec=f"{branch}:{branch}", force=True, progress=progress)
                    print(a[0].summary)
                    print(a[0].flags)
                    print(a[0].remote_ref_string)
                    print()
                logger.info(f"Branch {branch} uploaded successfully.")

        newRepo.edit(default_branch="master")

    def uploadMonoRepo(self, githubRepoName ,path = "./repos/MonoRepo", disableScanning = False):
        localRepo = git.Repo(path)
        logger.info(f"Uploading {githubRepoName} to the {githubRepoName}...")
        githubRepo = self.getOrCreateRepo(githubRepoName)
        if disableScanning or True:
            self.deactivatePushProtection(githubRepo)
        secrets = {"GITLABTOKEN": self.__gitlabToken}
        self.createSecrets(githubRepo,secrets)
        remote_url = githubRepo.clone_url.replace("https://", f"https://{self.__githubToken}@")
        self.reset_remote_origin(localRepo, remote_url)
        existingBranches = [b.name for b in githubRepo.get_branches()]
        for branch in localRepo.branches:
            if branch.name in existingBranches:
                logger.info(f"Branch {branch.name} already exists in the target repository.")
            else:
                logger.info(f"Uploading {branch.name} branch...")
                with (PushProgress() as progress):
                    a = localRepo.remote(name="origin").push(refspec=f"{branch.name}:{branch.name}", force=True, progress=progress)
                    print(a[0].summary)
                    print(a[0].flags)
                    print(a[0].remote_ref_string)
                    print()
                logger.info(f"Branch {branch.name} uploaded successfully.")
        githubRepo.edit(default_branch="master")
        if disableScanning or True:
            self.activatePushProtection(githubRepo)

    def deactivatePushProtection(self,repo):
        headers = {
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {self.__githubToken}",
            "X-GitHub-Api-Version": "2022-11-28"
        }
        payload = {
            "security_and_analysis": {
                "secret_scanning_push_protection": {"status": "disabled"},
            }
        }

        url = repo.url
        response = requests.patch(
            url,
            headers=headers,
            json=payload
        )
        if response.status_code == 200:
            logger.info("Push protection deactivated successfully.")
        else:
            logger.warning("Push protection deactivation failed. Push might not be possible. Either deactivate manually or push manually and remove blocked blobs.")

    def activatePushProtection(self,repo):
        headers = {
            "Accept": "application/vnd.github+json",
            "Authorization": f"Bearer {self.__githubToken}",
            "X-GitHub-Api-Version": "2022-11-28"
        }
        payload = {
            "security_and_analysis": {
                "secret_scanning_push_protection": {"status": "enabled"},
            }
        }

        url = repo.url
        response = requests.patch(
            url,
            headers=headers,
            json=payload
        )
        if response.status_code == 200:
            logger.info("Push protection activated successfully.")
        else:
            logger.warning("Push protection activation failed.")

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

    def set_upstream_for_branches(self,repo, remote_name="origin"):
        """
        Setzt den Upstream-Branch für alle lokalen Branches, die keinen Upstream haben.

        :param repo_path: Pfad zum lokalen Git-Repository
        :param remote_name: Name des Remote-Repositories (Standard: 'origin')
        """
        try:
            # Über alle lokalen Branches iterieren
            for branch in repo.branches:
                if branch.tracking_branch() is None:  # Prüfen, ob kein Upstream-Branch gesetzt ist
                    remote_branch = f"{remote_name}/{branch.name}"
                    repo.git.branch("--set-upstream-to", remote_branch, branch.name)
                    print(f"Upstream-Branch für '{branch.name}' auf '{remote_branch}' gesetzt.")
                else:
                    print(f"Upstream-Branch für '{branch.name}' ist bereits gesetzt.")
        except Exception as e:
            print(f"Fehler beim Setzen der Upstream-Branches: {e}")

    def dockerImageMigration(self,namespace,reponame,images):
        images = ",".join(images)
        action ="name: Migrate Docker Images\n"
        action+="on:\n"
        action+="  workflow_dispatch:\n"
        action+="jobs:\n"
        action+="  docker-migration:\n"
        action+="    runs-on: ubuntu-latest\n"
        action+="    env:\n"
        action+='      GITLAB_USERNAME: "David.Blum"\n' #TODO: make this dynamic
        action+="      GITLABTOKEN: ${{ secrets.GITLABTOKEN }}\n"
        action+=f'      GITLAB_REPO: "{(namespace+"/" + reponame).lower()}"\n'
        action+=f'      IMAGES_LIST: "{images}"\n'
        action+="      GHCR_PAT: ${{ secrets.GHCR_PAT }}\n"
        action+='      GHCR_REPO_OWNER: "davidblm"\n'
        action+="    steps:\n"
        action+="      - name: Log in to GitLab\n"
        action+="        run: |\n"
        action+='          docker login https://git.rwth-aachen.de/ -u "$GITLAB_USERNAME" -p "$GITLABTOKEN"\n'
        action+="      - name: Log in to GitHub\n"
        action+="        run: |\n"
        action+='          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin\n'
        action+="      - name: Migrate Docker images\n"
        action+="        run: |\n"
        action+='          IFS="," read -ra IMAGES <<< "$IMAGES_LIST"\n'
        action+='          for IMAGE in "${IMAGES[@]}"; do\n'
        action+='            GITLAB_IMAGE="registry.git.rwth-aachen.de/$GITLAB_REPO/$IMAGE"\n'
        action+='            LOWERCASE_IMAGE=$(echo "$IMAGE" | tr "[:upper:]" "[:lower:]")\n'
        action+='            GHCR_IMAGE="ghcr.io/$GHCR_REPO_OWNER/$LOWERCASE_IMAGE"\n'
        action+='            echo "Pulling image from GitLab: $GITLAB_IMAGE"\n'
        action+='            docker pull "$GITLAB_IMAGE"\n'
        action+='            echo "Tagging image for GHCR: $GHCR_IMAGE"\n'
        action+='            docker tag "$GITLAB_IMAGE" "$GHCR_IMAGE"\n'
        action+='            echo "Pushing image to GHCR: $GHCR_IMAGE"\n'
        action+='            docker push "$GHCR_IMAGE"\n'
        action+='            echo "Removing local image: $GHCR_IMAGE"\n'
        action+='            docker rmi -f $(docker images -q) || true\n'
        action+='          done\n'

        file_path = f"repos/{reponame}/.github/workflows/image.yml"
        folder_path = os.path.dirname(file_path)

        # Ordner erstellen, falls sie nicht existieren
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Ordner '{folder_path}' wurde erstellt.")

        # Datei schreiben
        with open(file_path, 'w') as file:
            file.write(action)
            print(f"Datei '{file_path}' wurde erfolgreich geschrieben.")
        repo = git.Repo("./repos/" + reponame)
        repo.git.add(all=True)
        repo.index.commit("Added Docker image migration workflow")

    def dockerImageMigrationMonorepo(self, architecture, targetRepo):
        action = "name: Migrate Docker Images\n"
        action += "on:\n"
        action += "  workflow_dispatch:\n"
        action += "jobs:\n"
        action += "  docker-migration:\n"
        action += "    runs-on: ubuntu-latest\n"
        action += "    env:\n"
        action += '      GITLAB_USERNAME: "David.Blum"\n'  # TODO: make this dynamic
        action += "      GITLABTOKEN: ${{ secrets.GITLABTOKEN }}\n"
        action += "      GHCR_PAT: ${{ secrets.GHCR_PAT }}\n"
        action += '      GHCR_REPO_OWNER: "davidblm"\n'
        action += "    steps:\n"
        action += "      - name: Log in to GitLab\n"
        action += "        run: |\n"
        action += '          docker login https://git.rwth-aachen.de/ -u "$GITLAB_USERNAME" -p "$GITLABTOKEN"\n'
        action += "      - name: Log in to GitHub\n"
        action += "        run: |\n"
        action += '          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin\n'
        for repoId in architecture.keys():
            images = ",".join(architecture[repoId]["DockerImages"])
            GITLAB_REPO = (architecture[repoId]["Namespace"] + "/" + architecture[repoId]["Name"]).lower()
            action += f"      - name: Migrate Docker images from {architecture[repoId]["Name"]}\n"
            action += "        run: |\n"
            action += f'          IFS="," read -ra IMAGES <<< "{images}"\n'
            action += '          for IMAGE in "${IMAGES[@]}"; do\n'
            action += f'            GITLAB_IMAGE="registry.git.rwth-aachen.de/{GITLAB_REPO}/$IMAGE"\n'
            action += '            LOWERCASE_IMAGE=$(echo "$IMAGE" | tr "[:upper:]" "[:lower:]")\n'
            action += '            GHCR_IMAGE="ghcr.io/$GHCR_REPO_OWNER/$LOWERCASE_IMAGE"\n'
            action += '            echo "Pulling image from GitLab: $GITLAB_IMAGE"\n'
            action += '            docker pull "$GITLAB_IMAGE"\n'
            action += '            echo "Tagging image for GHCR: $GHCR_IMAGE"\n'
            action += '            docker tag "$GITLAB_IMAGE" "$GHCR_IMAGE"\n'
            action += '            echo "Pushing image to GHCR: $GHCR_IMAGE"\n'
            action += '            docker push "$GHCR_IMAGE"\n'
            action += '            echo "Removing local image: $GHCR_IMAGE"\n'
            action += '            docker rmi -f $(docker images -q) || true\n'
            action += '          done\n'

        file_path = f"repos/{targetRepo}/.github/workflows/image.yml"
        folder_path = os.path.dirname(file_path)

        # Ordner erstellen, falls sie nicht existieren
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Ordner '{folder_path}' wurde erstellt.")

        # Datei schreiben
        with open(file_path, 'w') as file:
            file.write(action)
            print(f"Datei '{file_path}' wurde erfolgreich geschrieben.")
        repo = git.Repo("./repos/" + targetRepo)
        repo.git.add(all=True)
        repo.index.commit("Added Docker image migration workflow")


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


    def new_message_handler__disabled(self):
        """
        :return:
            a progress handler suitable for handle_process_output(), passing lines on to this Progress
            handler in a suitable format"""
        def handler(line):
            print(line.rstrip())
            return self._parse_progress_line(line.rstrip())
        return handler
