import os

import git

from pipelineMigration.GithubSubtreeConverter import GithubSubTreeConverter
from pipelineMigration.GitlabCIImporter import GitlabCIImporter
from pipelineMigration.GithubConverter import GithubActionConverter


def writeStringToFile( file_path, content):
    with open(file_path, 'w') as file:
        file.write(content)

#ToDo: Add for Subtree variant
def GitlabToGithub(gitlabFilePath: str, githubFilePath: str, name: str = "pipeline", secrets: list[str] = []) -> None:
    """
    This function migrates a file from GitLab to GitHub.

    :param gitlabFilePath: The path to the input GitLab file.
    :param githubFilePath: The path to the output GitHub file.
    :param name: The name of the pipeline.
    :param secrets: A list of names for secrets to be used in the pipeline.
    """
    file = open(gitlabFilePath+ "/.gitlab-ci.yml",'r')
    pipeline = GitlabCIImporter().getPipeline(file)
    file.close()

    repo = git.Repo(gitlabFilePath)
    GithubActionConverter.process_settings_files(gitlabFilePath)
    repo.git.add(all=True)
    repo.index.commit("Changed maven settings to private token")

    pipelineConverter = GithubActionConverter(pipeline)
    convertedPipeline = pipelineConverter.parsePipeline(name, secrets)

    file_path = f"{gitlabFilePath}/.github/workflows/main.yml"
    folder_path = os.path.dirname(file_path)

    # Ordner erstellen, falls sie nicht existieren
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")

    writeStringToFile(file_path, convertedPipeline)


    repo.git.add(all=True)
    repo.index.commit("Migrated pipeline from Gitlab to Github")


def GitlabToGithubSubtree(gitlabRepos, githubFilePath, githubRepoPrefix,secrets):
    subTreeRepo = git.Repo(githubFilePath)
    file_path_base = f"{githubFilePath}/.github/workflows/"
    folder_path = os.path.dirname(file_path_base)



    # Ordner erstellen, falls sie nicht existieren
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)
        print(f"Ordner '{folder_path}' wurde erstellt.")
    for repo in gitlabRepos:
        path, id = repo
        file = open(path + "/.gitlab-ci.yml", 'r')
        name = path.split("/")[-1]
        pipeline = GitlabCIImporter().getPipeline(file)
        file.close()
        pipelineConverter = GithubSubTreeConverter(pipeline, githubRepoPrefix[name], id)
        convertedPipeline = pipelineConverter.parsePipeline(name, secrets[name])

        file_path = file_path_base + name + ".yml"
        writeStringToFile(file_path, convertedPipeline)
        subTreeRepo.git.add(all=True)
        subTreeRepo.index.commit(f"Migrated pipeline of {name} from Gitlab to Github")


if __name__ == '__main__':
    gitlabFilePath = ".gitlab-ci.yml"
    githubFilePath = ".main.yml"
    GitlabToGithub(gitlabFilePath,githubFilePath,"pipeline", ["GitlabToken","URL","ID"])