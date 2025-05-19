import logging
import os

import git
from tqdm import tqdm

from src.Architecture import Architecture
from src.Config import Config
from src.pipelineMigration.GithubConverter import GithubActionConverter
from src.pipelineMigration.GithubSubtreeConverter import GithubSubTreeConverter
from src.pipelineMigration.GitlabCIImporter import GitlabCIImporter


def writeStringToFile(file_path, content):
    with open(file_path, 'w') as file:
        file.write(content)


# ToDo: Test
def GitlabToGithub(repoID: str, architecture: Architecture, config: Config, name: str = "") -> None:
    """
    This function migrates a Pipeline from GitLab to GitHub.
    :param repoID: The ID of the repository to be migrated.
    :param architecture: The architecture object
    :param config: The configuration object.
    :param name: The name of the pipeline.
    :param secrets: A list of names for secrets to be used in the pipeline.
    """
    repo = architecture.get_repo_by_ID(repoID)
    # Open the Gitlab CI file and parse it
    file = open(os.path.join(repo.path, "/.gitlab-ci.yml"), 'r')
    pipeline = GitlabCIImporter().getPipeline(file)
    file.close()

    # Change the image names in the pipeline to the migrated registry ones if necessary
    pipeline = changeToUpdatedImages(pipeline, architecture, config, repoID)

    # Open the git repository
    repo_git = repo.get_repo()
    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files(repo.path)
    repo_git.git.add(all=True)
    repo_git.index.commit("Changed maven settings to private token")

    # Convert the pipeline to Github Actions format
    pipelineConverter = GithubActionConverter(pipeline)
    convertedPipeline = pipelineConverter.parse_pipeline(name, repo.secrets)
    file_path = os.path.join(repo.path, f"./.github/workflows/{name}.yml")
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
        repo = architecture.get_repo_by_ID(repoID)
        if not repo.images:
            continue

        for image in repo.images.keys():
            if not image.startswith(":"):
                url = ("registry." + config.url.replace("https://",
                                                        "") + repo.namespace + "/" + repo.name + "/" + image).lower()
                newNames[url] = "ghcr.io/" + config.targetUser.lower() + "/" + repo.name + "/" + image
            else:
                # fullImageNames.append(("registry." + config.url.replace("https://", "") + architecture[repoID][
                #    "Namespace"] + "/" + architecture[repoID]["Name"] + image).lower())

                url = ("registry." + config.url.replace("https://",
                                                        "") + repo.namespace + "/" + repo.name + image).lower()
                newNames[url] = "ghcr.io/" + config.targetUser.lower() + "/" + repo.name + "/" + image

    # Change the image names in the pipeline object
    for _, job in pipeline.jobs.items():
        if job.image in newNames.keys():
            job.image = newNames[job.image]
    return pipeline


def GitlabToGithubSubtree(repoIDS, architecture: Architecture, config: Config,
                          rebuild=False):
    """
        Converts Gitlab pipelines to Github Actions pipelines for all repositories in the subtree monorepo and commits the changes.
    :param repoIDS: RepoIDS of the contained repos
    :param architecture: Architecture object
    :param config: Config object
    :param secrets: Secrets to be used in the pipeline
    """
    logger = logging.getLogger(__name__)
    github_file_path = os.path.join(os.getcwd(), "repos", config.monorepoName)
    print(github_file_path)
    subtree_repo = git.Repo(github_file_path)
    file_path_base = os.path.join(github_file_path, ".github", "workflows")

    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files(github_file_path)
    subtree_repo.git.add(all=True)
    subtree_repo.index.commit(f"Changed maven settings to private token")

    github_repo_prefix = {}  # Path to each subtree in the monorepo
    monorepoNamespace = config.monorepoNamespace.split("/")
    for repoID in architecture.repoIDs:
        repo = architecture.get_repo_by_ID(repoID)
        repoNamespace = "/".join([i for i in repo.namespace.split("/") if i not in monorepoNamespace])
        github_repo_prefix[repo.name] = os.path.join(repoNamespace, repo.name)

    # Create Github action folder
    if not os.path.exists(file_path_base):
        os.makedirs(file_path_base)
        logger.info(f"Ordner '{file_path_base}' wurde erstellt.")

    branches_to_be_migrated = {}
    iterations = 0
    for repoID in repoIDS:
        repo = architecture.get_repo_by_ID(repoID)
        branches_to_be_migrated[repoID] = repo.get_branches_to_be_migrated()
        iterations += len(branches_to_be_migrated[repoID])

    with tqdm(total=iterations, desc="Migrating pipelines", unit="branch") as progress:
        # Migrate all contained repos
        for repoID in repoIDS:
            repo = architecture.get_repo_by_ID(repoID)
            # Check, which branches were migrated for the repo
            multiple = len(branches_to_be_migrated[str(repoID)]) > 1
            # Iterate over all migrated branches
            for branch in branches_to_be_migrated[str(repoID)]:
                # Chose path to gitlab pipeline according to structure
                if multiple:
                    path = os.path.join(github_file_path, github_repo_prefix[repo.name], branch)
                else:
                    path = os.path.join(github_file_path, github_repo_prefix[repo.name])

                # Import gitlab pipeline
                try:
                    file = open(os.path.join(path, '.gitlab-ci.yml'), 'r')
                except FileNotFoundError:
                    logger.info(f"No .gitlab-ci.yml found for repo {repo.name} in branch {branch}. Skipping.")
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
                    pipelineConverter = GithubSubTreeConverter(architecture, pipeline, github_repo_prefix[repo.name],
                                                               repoID,
                                                               rebuild=rebuild)
                    convertedPipeline = pipelineConverter.parse_pipeline(repo.name, repo.secrets)
                    file_path = os.path.join(file_path_base, repo.name + ".yml")
                else:
                    pipelineConverter = GithubSubTreeConverter(architecture, pipeline,
                                                               github_repo_prefix[repo.name] + "/" + branch,
                                                               repoID,
                                                               rebuild=rebuild)
                    convertedPipeline = pipelineConverter.parse_pipeline(repo.name + "_" + branch, repo.secrets)
                    file_path = os.path.join(file_path_base, repo.name + "_" + branch + ".yml")
                # Write and commit action
                writeStringToFile(file_path, convertedPipeline)
                subtree_repo.git.add(all=True)
                subtree_repo.index.commit(f"Migrated pipeline of {repo.name} and branch {branch} from Gitlab to Github")
                progress.update(1)


if __name__ == '__main__':
    gitlabFilePath = ".gitlab-ci.yml"
    githubFilePath = ".main.yml"
    GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
