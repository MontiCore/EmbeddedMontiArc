import logging
import os

import git
import requests
from git import RemoteProgress
from tqdm import tqdm

from src.gitMigration.Uploader import Uploader
from src.gitMigration.Git import Git

logger = logging.getLogger(__name__)

class GithubUploader(Git, Uploader):
    def __init__(self, config, architecture):
        super().__init__()
        Uploader.__init__(config, architecture)

    def deactivate_push_protection(self, repo):
        """
            Deactivates push protection for the given GitHub repository.
        :param repo: Github repo object
        :return:
        """
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
            logger.warning(
                "Push protection deactivation failed. Push might not be possible. Either deactivate manually or push manually and remove blocked blobs.")

    def activate_push_protection(self, repo):
        """
            Activates Push Protection for the given GitHub repository.
        :param repo: Github repository object
        """
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


    # ToDo: Remove once MonoRepo variant is tested If needed migrate to new architecture
    """
    def dockerImageMigration(self, architecture, repoID):
        images = ",".join(architecture[repoID]["DockerImages"])
        action = "name: Migrate Docker Images\n"
        action += "on:\n"
        action += "  workflow_dispatch:\n"
        action += "jobs:\n"
        action += "  docker-migration:\n"
        action += "    runs-on: ubuntu-latest\n"
        action += "    env:\n"
        action += f'      GITLAB_USERNAME: "{self.config.sourceUser}"\n'
        action += "      GITLABTOKEN: ${{ secrets.GITLABTOKEN }}\n"
        action += f'      GITLAB_REPO: "{(architecture[repoID]["Namespace"] + "/" + architecture[repoID]["Name"]).lower()}"\n'
        action += f'      IMAGES_LIST: "{images}"\n'
        action += "      GHCR_PAT: ${{ secrets.GHCR_PAT }}\n"
        action += '      GHCR_REPO_OWNER: "davidblm"\n'
        action += "    steps:\n"
        action += "      - name: Log in to GitLab\n"
        action += "        run: |\n"
        action += '          docker login https://git.rwth-aachen.de/ -u "$GITLAB_USERNAME" -p "$GITLABTOKEN"\n'
        action += "      - name: Log in to GitHub\n"
        action += "        run: |\n"
        action += '          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin\n'
        action += "      - name: Migrate Docker images\n"
        action += "        run: |\n"
        action += '          IFS="," read -ra IMAGES <<< "$IMAGES_LIST"\n'
        action += '          for IMAGE in "${IMAGES[@]}"; do\n'
        action += '            GITLAB_IMAGE="registry.git.rwth-aachen.de/$GITLAB_REPO/$IMAGE"\n'
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

        file_path = f"repos/{architecture[repoID]["Name"]}/.github/workflows/image.yml"
        folder_path = os.path.dirname(file_path)

        # Ordner erstellen, falls sie nicht existieren
        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Ordner '{folder_path}' wurde erstellt.")

        # Datei schreiben
        with open(file_path, 'w') as file:
            file.write(action)
            print(f"Datei '{file_path}' wurde erfolgreich geschrieben.")
        repo = git.Repo("./repos/" + architecture[repoID]["Name"])
        repo.git.add(all=True)
        repo.index.commit("Added Docker image migration workflow")
    """

    def docker_image_migration_monorepo(self, repos_to_be_migrated=None):
        """
            Adds a new GitHub action to migrate Docker images from GitLab to GitHub.
        :param architecture: Architecture object
        :param repos_to_be_migrated: List of repositories to be migrated. If None, all repositories are migrated.
        :return:
        """

        if repos_to_be_migrated is None:
            repos_to_be_migrated = self.architecture.repos.keys()
        action = "name: Migrate Docker Images\n"
        action += "on:\n"
        action += "  workflow_dispatch:\n"
        action += "jobs:\n"
        action += "  docker-migration:\n"
        action += "    runs-on: ubuntu-latest\n"
        action += "    env:\n"
        action += f'      GITLAB_USERNAME: "{self.config.sourceUser}"\n'
        action += "      GITLABTOKEN: ${{ secrets.GITLABTOKEN }}\n"
        action += "      GHCR_PAT: ${{ secrets.GHCR_PAT }}\n"
        # action += '      GHCR_REPO_OWNER: "davidblm"\n'
        # action += "      GHCR_REPO_OWNER: ${{ github.actor | toLowerCase}}\n" #ToDo: Add automatic username
        action += "    steps:\n"
        action += "      - name: Log in to GitLab\n"
        action += "        run: |\n"
        action += '          docker login https://git.rwth-aachen.de/ -u "$GITLAB_USERNAME" -p "$GITLABTOKEN"\n'
        action += "      - name: Log in to GitHub\n"
        action += "        run: |\n"
        action += '          echo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin\n'
        for repoId in repos_to_be_migrated:
            repo = self.architecture.get_repo_by_ID(repoId)
            images = ",".join(repo.images)
            gitlab_repo = (repo.namespace + "/" + repo.name).lower()
            action += f"      - name: Migrate Docker images from {repo.name}\n"
            action += "        run: |\n"
            action += '          LOWERCASE_OWNER=$(echo "${{ github.repository_owner }}" | tr "[:upper:]" "[:lower:]")\n'
            action += f'          IFS="," read -ra IMAGES <<< "{images}"\n'
            action += '          for IMAGE in "${IMAGES[@]}"; do\n'
            # action += f'            GITLAB_IMAGE="registry.git.rwth-aachen.de/{gitlab_repo}/$IMAGE"\n'
            action += "            if [[ $IMAGE == :* ]]; then\n"
            action += f'              GITLAB_IMAGE="registry.git.rwth-aachen.de/{gitlab_repo}$IMAGE"\n'
            action += "            else\n"
            action += f'              GITLAB_IMAGE="registry.git.rwth-aachen.de/{gitlab_repo}/$IMAGE"\n'
            action += "            fi\n"
            action += '            LOWERCASE_IMAGE=$(echo "$IMAGE" | tr "[:upper:]" "[:lower:]")\n'
            action += "            if [[ $IMAGE == :* ]]; then\n"
            action += f'              GHCR_IMAGE="ghcr.io/$LOWERCASE_OWNER/{repo.name.lower()}$LOWERCASE_IMAGE"\n'
            action += "            else\n"
            action += f'               GHCR_IMAGE="ghcr.io/$LOWERCASE_OWNER/{repo.name.lower()}/$LOWERCASE_IMAGE"\n'
            action += "            fi\n"
            action += '            echo "Pulling image from GitLab: $GITLAB_IMAGE"\n'
            action += '            docker pull "$GITLAB_IMAGE"\n'
            action += '            echo "Tagging image for GHCR: $GHCR_IMAGE"\n'
            action += '            docker tag "$GITLAB_IMAGE" "$GHCR_IMAGE"\n'
            action += '            echo "Pushing image to GHCR: $GHCR_IMAGE"\n'
            action += '            docker push "$GHCR_IMAGE"\n'
            action += '            echo "Removing local image: $GHCR_IMAGE"\n'
            action += '            docker rmi -f $(docker images -q) || true\n'
            action += '          done\n'
        target_repo = self.config.monorepoName
        file_path = os.path.join(os.getcwd(), "repos", target_repo, ".github", "workflows", "image.yml")
        folder_path = os.path.dirname(file_path)
        if not os.path.exists(os.path.join(os.getcwd(), "repos", target_repo)):
            logger.error(f"Repository '{target_repo}' nicht gefunden.")
            exit(1)

        if not os.path.exists(folder_path):
            os.makedirs(folder_path)
            print(f"Ordner '{folder_path}' wurde erstellt.")
        with open(file_path, 'w') as file:
            file.write(action)
            print(f"Datei '{file_path}' wurde erfolgreich geschrieben.")
        repo = git.Repo(os.path.join(os.getcwd(), "repos", target_repo))
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
        Can be used to print the output of the git push command directly for debugging. To do so remove the __disabled from the name
        :return:
            A progress handler suitable for handle_process_output(), passing lines on to this Progress
            handler in a suitable format"""

        def handler(line):
            print(line.rstrip())
            return self._parse_progress_line(line.rstrip())

        return handler