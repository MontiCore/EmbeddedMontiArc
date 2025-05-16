import logging
import os

import git
from tqdm import tqdm

from src.Config import Config
from src.pipelineMigration.GithubSubtreeConverter import GithubSubTreeConverter
from src.pipelineMigration.GitlabCIImporter import GitlabCIImporter
from src.pipelineMigration.GithubConverter import GithubActionConverter


def writeStringToFile(file_path, content):
    with open(file_path, 'w') as file:
        file.write(content)


def GitlabToGithub(repoID: str, architecture, config: Config, name: str = "pipeline", secrets: list[str] = []) -> None:
    """
    This function migrates a Pipeline from GitLab to GitHub.
    :param repoID: The ID of the repository to be migrated.
    :param architecture: The architecture object
    :param config: The configuration object.
    :param name: The name of the pipeline.
    :param secrets: A list of names for secrets to be used in the pipeline.
    """
    repo_path = os.path.join(os.getcwd(), "repos", architecture[repoID]["Name"], )
    # Open the Gitlab CI file and parse it
    file = open(os.path.join(repo_path, "/.gitlab-ci.yml"), 'r')
    pipeline = GitlabCIImporter().getPipeline(file)
    file.close()

    # Change the image names in the pipeline to the migrated registry ones if necessary
    pipeline = changeToUpdatedImages(pipeline, architecture, config, repoID)

    # Open the git repository
    repo = git.Repo(repo_path)
    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files(repo_path)
    repo.git.add(all=True)
    repo.index.commit("Changed maven settings to private token")

    # Convert the pipeline to Github Actions format
    pipelineConverter = GithubActionConverter(pipeline)
    convertedPipeline = pipelineConverter.parse_pipeline(name, secrets)
    file_path = os.path.join(repo_path, f"./.github/workflows/{name}.yml")
    folder_path = os.path.dirname(file_path)
    # Ordner erstellen, falls sie nicht existieren
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")
    writeStringToFile(file_path, convertedPipeline)
    # Commit the new action
    repo.git.add(all=True)
    repo.index.commit("Migrated pipeline from Gitlab to Github")


def changeToUpdatedImages(pipeline, architecture, config, repoIDS):
    """
    Changes the image names in the pipeline to the updated ones.
    :param pipeline: The pipeline object
    :param architecture: The architecture object
    :param repoID: The ID of the repository
    """
    # Get the full image names from the architecture
    # ToDo: Save already migrated images in config so that they are changed later on
    newNames = {}
    if type(repoIDS) == str:
        repoIDS = [repoIDS]
    for repoID in repoIDS:
        if "DockerImages" not in architecture[repoID] or architecture[repoID]["DockerImages"] is None:
            continue

        for image in architecture[repoID]["DockerImages"]:
            if not image.startswith(":"):
                url = ("registry." + config.url.replace("https://", "") + architecture[repoID][
                    "Namespace"] + "/" + architecture[repoID]["Name"] + "/" + image).lower()
                newNames[url] = "ghcr.io/" + config.targetUser.lower() + "/" + architecture[repoID][
                    "Name"] + "/" + image
            else:
                # fullImageNames.append(("registry." + config.url.replace("https://", "") + architecture[repoID][
                #    "Namespace"] + "/" + architecture[repoID]["Name"] + image).lower())

                url = ("registry." + config.url.replace("https://", "") + architecture[repoID][
                    "Namespace"] + "/" + architecture[repoID]["Name"] + image).lower()
                newNames[url] = "ghcr.io/" + config.targetUser.lower() + "/" + architecture[repoID][
                    "Name"] + "/" + image

    # Change the image names in the pipeline object
    for _, job in pipeline.jobs.items():
        if job.image in newNames.keys():
            job.image = newNames[job.image]
    return pipeline


def GitlabToGithubSubtree(repoIDS, architecture, config: Config, github_file_path, github_repo_prefix, secrets,
                          rebuild=False):
    """
        Converts Gitlab pipelines to Github Actions pipelines for all repositories in the subtree monorepo and commits the changes.
    :param repoIDS: RepoIDS of the contained repos
    :param architecture: Architecture object
    :param config: Config object
    :param github_file_path: Path to the Github repo
    :param github_repo_prefix: Path to each repo in the monorepo
    :param secrets: Secrets to be used in the pipeline
    """
    logger = logging.getLogger(__name__)

    subtree_repo = git.Repo(github_file_path)

    file_path_base = os.path.join(github_file_path, ".github", "workflows")
    folder_path = os.path.dirname(file_path_base)
    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files(github_file_path)
    subtree_repo.git.add(all=True)
    subtree_repo.index.commit(f"Changed maven settings to private token")

    # Create Github action folder
    if not os.path.exists(file_path_base):
        os.makedirs(file_path_base)
        logger.info(f"Ordner '{file_path_base}' wurde erstellt.")

    branches_to_be_migrated = {}
    iterations = 0
    for repoID in repoIDS:
        if architecture[repoID]["Branches"] is None:
            branches_to_be_migrated[str(repoID)] = architecture[repoID]["StaleBranches"]
        elif architecture[repoID]["StaleBranches"] is None:
            branches_to_be_migrated[str(repoID)] = architecture[repoID]["Branches"]
        else:
            branches_to_be_migrated[str(repoID)] = list(
                set(architecture[repoID]["Branches"]).union(set(architecture[repoID]["StaleBranches"])))
        iterations += len(branches_to_be_migrated[str(repoID)])

    with tqdm(total=iterations, desc="Migrating pipelines", unit="branch") as progress:
        # Migrate all contained repos
        for repoID in repoIDS:
            # Check, which branches were migrated for the repo

            multiple = len(branches_to_be_migrated[str(repoID)]) > 1
            # Iterate over all migrated branches
            for branch in branches_to_be_migrated[str(repoID)]:
                # Chose path to gitlab pipeline according to structure
                name = architecture[repoID]["Name"]
                if multiple:
                    path = os.path.join(github_file_path, github_repo_prefix[name], branch)
                else:
                    path = os.path.join(github_file_path, github_repo_prefix[name])

                # Import gitlab pipeline
                try:
                    file = open(os.path.join(path, '.gitlab-ci.yml'), 'r')
                except FileNotFoundError:
                    logger.info(f"No .gitlab-ci.yml found for repo {name} in branch {branch}. Skipping.")
                    progress.update(1)
                    continue
                pipeline = GitlabCIImporter().getPipeline(file)
                file.close()

                jobs_to_delete = []
                for job in pipeline.jobs.values():
                    if job.only:
                        if type(job.only) == list:
                            if branch not in job.only:
                                jobs_to_delete.append(job.name)
                    if job.exc:
                        if type(job.exc) == list:
                            if branch in job.exc:
                                jobs_to_delete.append(job.name)
                for job in jobs_to_delete:
                    pipeline.delete_job(job)

                pipeline = changeToUpdatedImages(pipeline, architecture, config, repoIDS)
                # Convert the pipeline to Github Actions format, depending on the number of branches
                if len(branches_to_be_migrated[str(repoID)]) <= 1:
                    pipelineConverter = GithubSubTreeConverter(pipeline, github_repo_prefix[name], repoID,
                                                               rebuild=rebuild)
                    convertedPipeline = pipelineConverter.parse_pipeline(name, secrets[name])
                    file_path = os.path.join(file_path_base, name + ".yml")
                else:
                    pipelineConverter = GithubSubTreeConverter(pipeline, github_repo_prefix[name] + "/" + branch,
                                                               repoID,
                                                               rebuild=rebuild)
                    convertedPipeline = pipelineConverter.parse_pipeline(name + "_" + branch, secrets[name])
                    file_path = os.path.join(file_path_base, name + "_" + branch + ".yml")
                # Write and commit action
                writeStringToFile(file_path, convertedPipeline)
                subtree_repo.git.add(all=True)
                subtree_repo.index.commit(f"Migrated pipeline of {name} and branch {branch} from Gitlab to Github")
                progress.update(1)


if __name__ == '__main__':
    gitlabFilePath = ".gitlab-ci.yml"
    githubFilePath = ".main.yml"
    GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
