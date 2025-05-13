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
    This function migrates a file from GitLab to GitHub.

    :param gitlabFilePath: The path to the input GitLab file.
    :param name: The name of the pipeline.
    :param secrets: A list of names for secrets to be used in the pipeline.
    """
    file = open("./repos/" + architecture[repoID]["Name"] + "/.gitlab-ci.yml", 'r')
    pipeline = GitlabCIImporter().getPipeline(file)
    file.close()

    pipeline = changeToUpdatedImages(pipeline, architecture, config, repoID)

    repo = git.Repo("./repos/" + architecture[repoID]["Name"])
    GithubActionConverter.process_settings_files("./repos/" + architecture[repoID]["Name"])
    repo.git.add(all=True)
    repo.index.commit("Changed maven settings to private token")

    pipelineConverter = GithubActionConverter(pipeline)
    convertedPipeline = pipelineConverter.parsePipeline(name, secrets)

    file_path = f"{"./repos/" + architecture[repoID]["Name"]}/.github/workflows/main.yml"
    folder_path = os.path.dirname(file_path)

    # Ordner erstellen, falls sie nicht existieren
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")

    writeStringToFile(file_path, convertedPipeline)

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
                # fullImageNames.append(("registry." + config.url.replace("https://", "") + architecture[repoID][
                #    "Namespace"] + "/" + architecture[repoID]["Name"] + image).lower())
                pass

    for _, job in pipeline.jobs.items():
        if job.image in newNames.keys():
            job.image = newNames[job.image]
    return pipeline


def GitlabToGithubSubtree(repoIDS, architecture, config: Config, githubFilePath, githubRepoPrefix, secrets):
    subTreeRepo = git.Repo(githubFilePath)
    file_path_base = f"{githubFilePath}/.github/workflows/"
    folder_path = os.path.dirname(file_path_base)
    GithubActionConverter.process_settings_files(githubFilePath)
    subTreeRepo.git.add(all=True)
    subTreeRepo.index.commit(f"Changed maven settings to private token")

    # Ordner erstellen, falls sie nicht existieren
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")
    for repoID in repoIDS:
        branchesToBeMigrated = {}
        if architecture[repoID]["Branches"] is None:
            branchesToBeMigrated[str(repoID)] = architecture[repoID]["StaleBranches"]
        elif architecture[repoID]["StaleBranches"] is None:
            branchesToBeMigrated[str(repoID)] = architecture[repoID]["Branches"]
        else:
            branchesToBeMigrated[str(repoID)] = list(
                set(architecture[repoID]["Branches"]).union(set(architecture[repoID]["StaleBranches"])))
        multiple = len(branchesToBeMigrated[str(repoID)]) > 1
        for branch in branchesToBeMigrated[str(repoID)]:
            if multiple:
                path = githubFilePath + "/" + architecture[repoID]["Namespace"] + "/" + architecture[repoID][
                    "Name"] + "/" + branch
            else:
                path = "./repos/" + architecture[repoID]["Name"]
            name = architecture[repoID]["Name"]

            file = open(path + "/.gitlab-ci.yml", 'r')
            pipeline = GitlabCIImporter().getPipeline(file)
            file.close()
            pipeline = changeToUpdatedImages(pipeline, architecture, config, repoIDS)

            if len(branchesToBeMigrated[str(repoID)]) <= 1:
                pipelineConverter = GithubSubTreeConverter(pipeline, githubRepoPrefix[name], repoID)
                convertedPipeline = pipelineConverter.parsePipeline(name, secrets[name])
                file_path = file_path_base + name + ".yml"
            else:
                pipelineConverter = GithubSubTreeConverter(pipeline, githubRepoPrefix[name] + "/" + branch, repoID)
                convertedPipeline = pipelineConverter.parsePipeline(name + "_" + branch, secrets[name])
                file_path = file_path_base + name + "_" + branch + ".yml"

            writeStringToFile(file_path, convertedPipeline)
            subTreeRepo.git.add(all=True)
            subTreeRepo.index.commit(f"Migrated pipeline of {name} and branch {branch} from Gitlab to Github")


if __name__ == '__main__':
    gitlabFilePath = ".gitlab-ci.yml"
    githubFilePath = ".main.yml"
    GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
