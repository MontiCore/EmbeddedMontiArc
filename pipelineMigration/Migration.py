import os
from platform import architecture

import git

from Config import Config
from pipelineMigration.GithubSubtreeConverter import GithubSubTreeConverter
from pipelineMigration.GitlabCIImporter import GitlabCIImporter
from pipelineMigration.GithubConverter import GithubActionConverter


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

    # Open the Gitlab CI file and parse it
    file = open("./repos/" + architecture[repoID]["Name"] + "/.gitlab-ci.yml", 'r')
    pipeline = GitlabCIImporter().getPipeline(file)
    file.close()

    # Change the image names in the pipeline to the migrated registry ones if necessary
    pipeline = changeToUpdatedImages(pipeline, architecture, config, repoID)

    # Open the git repository
    repo = git.Repo("./repos/" + architecture[repoID]["Name"])
    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files("./repos/" + architecture[repoID]["Name"])
    repo.git.add(all=True)
    repo.index.commit("Changed maven settings to private token")

    # Convert the pipeline to Github Actions format
    pipelineConverter = GithubActionConverter(pipeline)
    convertedPipeline = pipelineConverter.parse_pipeline(name, secrets)
    file_path = f"{"./repos/" + architecture[repoID]["Name"]}/.github/workflows/main.yml"
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
    newNames = {}
    if type(repoIDS) == str:
        repoIDS = [repoIDS]
    for repoID in repoIDS:
        for image in architecture[repoID]["DockerImages"]:
            if not image.startswith(":"):
                url = ("registry." + config.url.replace("https://", "") + architecture[repoID][
                    "Namespace"] + "/" + architecture[repoID]["Name"] + "/" + image).lower()
                newNames[url] = "ghcr.io/" + config.targetUser.lower() + "/" + image
            else:
                # ToDo: Whats with the weird image?

                # fullImageNames.append(("registry." + config.url.replace("https://", "") + architecture[repoID][
                #    "Namespace"] + "/" + architecture[repoID]["Name"] + image).lower())
                pass

    # Change the image names in the pipeline object
    for _, job in pipeline.jobs.items():
        if job.image in newNames.keys():
            job.image = newNames[job.image]
    return pipeline


def GitlabToGithubSubtree(repoIDS, architecture, config: Config, github_file_path, github_repo_prefix, secrets):
    """
        Converts Gitlab pipelines to Github Actions pipelines for all repositories in the subtree monorepo and commits the changes.
    :param repoIDS: RepoIDS of the contained repos
    :param architecture: Architecture object
    :param config: Config object
    :param github_file_path: Path to the Github repo
    :param github_repo_prefix: Path to each repo in the monorepo
    :param secrets: Secrets to be used in the pipeline
    """

    subtree_repo = git.Repo(github_file_path)

    file_path_base = f"{github_file_path}/.github/workflows/"
    folder_path = os.path.dirname(file_path_base)
    # Converts the maven files to be compatible with the private token and commit changes
    GithubActionConverter.process_settings_files(github_file_path)
    subtree_repo.git.add(all=True)
    subtree_repo.index.commit(f"Changed maven settings to private token")

    # Create Github action folder
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")
    # Migrate all contained repos
    for repoID in repoIDS:
        # Check, which branches were migrated for the repo
        branchesToBeMigrated = {}
        if architecture[repoID]["Branches"] is None:
            branchesToBeMigrated[str(repoID)] = architecture[repoID]["StaleBranches"]
        elif architecture[repoID]["StaleBranches"] is None:
            branchesToBeMigrated[str(repoID)] = architecture[repoID]["Branches"]
        else:
            branchesToBeMigrated[str(repoID)] = list(
                set(architecture[repoID]["Branches"]).union(set(architecture[repoID]["StaleBranches"])))
        multiple = len(branchesToBeMigrated[str(repoID)]) > 1
        # Iterate over all migrated branches
        for branch in branchesToBeMigrated[str(repoID)]:
            # Chose path to gitlab pipeline according to structure
            if multiple:
                path = github_file_path + "/" + architecture[repoID]["Namespace"] + "/" + architecture[repoID][
                    "Name"] + "/" + branch
            else:
                path = "./repos/" + architecture[repoID]["Name"]
            name = architecture[repoID]["Name"]

            # Impoer gitlab pipeline
            file = open(path + "/.gitlab-ci.yml", 'r')
            pipeline = GitlabCIImporter().getPipeline(file)
            file.close()
            pipeline = changeToUpdatedImages(pipeline, architecture, config, repoIDS)
            # Convert the pipeline to Github Actions format, depending on the number of branches
            if len(branchesToBeMigrated[str(repoID)]) <= 1:
                pipelineConverter = GithubSubTreeConverter(pipeline, github_repo_prefix[name], repoID)
                convertedPipeline = pipelineConverter.parse_pipeline(name, secrets[name])
                file_path = file_path_base + name + ".yml"
            else:
                pipelineConverter = GithubSubTreeConverter(pipeline, github_repo_prefix[name] + "/" + branch, repoID)
                convertedPipeline = pipelineConverter.parse_pipeline(name + "_" + branch, secrets[name])
                file_path = file_path_base + name + "_" + branch + ".yml"
            # Write and commit action
            writeStringToFile(file_path, convertedPipeline)
            subtree_repo.git.add(all=True)
            subtree_repo.index.commit(f"Migrated pipeline of {name} and branch {branch} from Gitlab to Github")


if __name__ == '__main__':
    gitlabFilePath = ".gitlab-ci.yml"
    githubFilePath = ".main.yml"
    GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
