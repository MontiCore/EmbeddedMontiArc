import logging
import os

import git
from rich import print
from rich.console import Console
from rich.progress import Progress
from rich.table import Table

from migrationTool.migration_types import Architecture, Config
from migrationTool.pipeline_migration.DockerMigration import DockerMigration
from migrationTool.pipeline_migration.GithubConverter import GithubActionConverter
from migrationTool.pipeline_migration.GithubSubtreeConverter import (GithubSubTreeConverter, )
from migrationTool.pipeline_migration.GitlabCIImporter import GitlabCIImporter


def writeStringToFile(file_path, content):
  with open(file_path, "w") as file:
    file.write(content)


logger = logging.getLogger(__name__)


def GitlabToGithub(architecture: Architecture, config: Config, rebuild=False) -> None:
  """
    Converts Gitlab pipelines to Github Actions pipelines for all repositories in the subtree monorepo and commits
    the changes.
    :param architecture: Architecture object
    :param config: Config object
  """
  console = Console()
  branches_to_be_migrated = {}
  iterations = 0
  table = Table("Repository name", "Branch")
  for repoID in config.repoIDS:
    repo = architecture.get_repo_by_ID(repoID)
    branches_to_be_migrated[repoID] = repo.get_branches_to_be_migrated()
    for branch in branches_to_be_migrated[repoID]:
      table.add_row(repo.name, branch)
    iterations += len(branches_to_be_migrated[repoID])
  console.print(table)
  docker_migration = DockerMigration(architecture, config, "docker_images.txt")
  summary = {}
  dependencies = set()
  with Progress() as progress:
    task = progress.add_task("Migrating", total=iterations)
    for repoID in config.repoIDS:
      repo = architecture.get_repo_by_ID(repoID)
      github_file_path = os.path.join(os.getcwd(), "repos", repo.name)
      file_path_base = os.path.join(github_file_path, ".github", "workflows")

      # Converts the maven files to be compatible with the private token and commit changes
      GithubActionConverter.process_settings_files(github_file_path)
      repo.get_repo().git.add(all=True)
      repo.get_repo().index.commit(f"Changed maven settings to private token")

      # Create Github action folder
      if not os.path.exists(file_path_base):
        os.makedirs(file_path_base)
        logger.info(f"Ordner '{file_path_base}' wurde erstellt.")

      summary[repo.name] = {}
      # Iterate over all migrated branches
      for branch in branches_to_be_migrated[str(repoID)]:
        # Import gitlab pipeline
        try:
          file = open(os.path.join(github_file_path, ".gitlab-ci.yml"), "r")
        except FileNotFoundError:
          logger.warning(f"No .gitlab-ci.yml found for repo {repo.name} in branch {branch} under {path}. Skipping.")
          print(f"[red] No pipeline found for repo {repo.name} in branch {branch}. Skipping.[/red]")
          summary[repo.name][branch] = ":x:"
          progress.update(task, advance=1)
          continue
        pipeline = GitlabCIImporter().get_pipeline(file)
        file.close()

        new_dependencies, pipeline = changeToUpdatedImages(progress, docker_migration, pipeline)
        dependencies = dependencies.union(new_dependencies)
        # Convert the pipeline to Github Actions format, depending on the number of branches
        pipelineConverter = GithubActionConverter(architecture, pipeline, compatibleImages=docker_migration.nativeImage,
                                                  rebuild=rebuild, )
        convertedPipeline = pipelineConverter.parse_pipeline(repoID)
        file_path = os.path.join(file_path_base, repo.name + ".yml")
        print(f"[green]Converted pipeline of {repo.name} and branch {branch}")
        summary[repo.name][branch] = ":white_check_mark:"
        # Write and commit action
        writeStringToFile(file_path, convertedPipeline)
        repo.get_repo().git.add(all=True)
        repo.get_repo().index.commit(f"Migrated pipeline of {repo.name} and branch {branch} from Gitlab to Github")
        progress.update(task, advance=1)
  docker_migration.write_images_being_migrated()
  print()
  print("[bold]Summary of migration:")
  table = Table("Repository name", "Branch", "Migration successful")
  for repoID in config.repoIDS:
    repo = architecture.get_repo_by_ID(repoID)
    for branch in branches_to_be_migrated[repoID]:
      if branch in summary[repo.name]:
        table.add_row(repo.name, branch, summary[repo.name][branch])
      else:
        table.add_row(repo.name, branch, ":x:")
  console.print(table)
  if dependencies:
    print()
    print("[yellow]During the pipeline migration, dependencies with images from the following repos were detected:")
    for repo in dependencies:
      print(repo)
    print("[yellow]Please migrate them to the new registry as well, before running the actions.")


def changeToUpdatedImages(progress, docker_migration, pipeline):
  """
  Changes the image names in the pipeline to the updated ones.
  :param pipeline: The pipeline object
  :param docker_migration: Docker migration object
  :param pipeline: Pipline object
  """
  dependencies = set()
  # Change the image names in the pipeline object
  for _, job in pipeline.jobs.items():
    if job.image != "":
      new_dependency, job.image = docker_migration.get_new_Image(progress, job.image)
      if new_dependency:
        dependencies.add(new_dependency)
  return dependencies, pipeline


def GitlabToGithubSubtree(architecture: Architecture, config: Config, rebuild=False):
  """
  Converts Gitlab pipelines to Github Actions pipelines for all repositories in the subtree monorepo and commits
  the changes.
  :param architecture: Architecture object
  :param config: Config object
  """
  console = Console()
  github_file_path = os.path.join(os.getcwd(), "repos", config.monorepoName)
  subtree_repo = git.Repo(github_file_path)
  file_path_base = os.path.join(github_file_path, ".github", "workflows")

  # Converts the maven files to be compatible with the private token and commit changes
  GithubActionConverter.process_settings_files(github_file_path)
  subtree_repo.git.add(all=True)
  subtree_repo.index.commit(f"Changed maven settings to private token")

  github_repo_prefix = {}  # Path to each subtree in the monorepo
  monorepoNamespace = config.monorepoNamespace.split("/")
  for repoID in config.repoIDS:
    repo = architecture.get_repo_by_ID(repoID)
    repoNamespace = "/".join([i for i in repo.namespace.split("/") if i not in monorepoNamespace])
    github_repo_prefix[repo.name] = os.path.join(repoNamespace, repo.name)

  # Create Github action folder
  if not os.path.exists(file_path_base):
    os.makedirs(file_path_base)
    logger.info(f"Ordner '{file_path_base}' wurde erstellt.")
  print(f"Trying to convert following pipelines in monorepo at {github_file_path}...")

  branches_to_be_migrated = {}
  iterations = 0
  table = Table("Repository name", "Branch")
  for repoID in config.repoIDS:
    repo = architecture.get_repo_by_ID(repoID)
    branches_to_be_migrated[repoID] = repo.get_branches_to_be_migrated()
    for branch in branches_to_be_migrated[repoID]:
      table.add_row(repo.name, branch)
    iterations += len(branches_to_be_migrated[repoID])
  console.print(table)

  summary = {}
  docker_migration = DockerMigration(architecture, config, "docker_images.txt")
  dependencies = set()
  with Progress() as progress:
    task = progress.add_task("Migrating", total=iterations)
    # Migrate all contained repos
    for repoID in config.repoIDS:
      repo = architecture.get_repo_by_ID(repoID)
      summary[repo.name] = {}
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
          file = open(os.path.join(path, ".gitlab-ci.yml"), "r")
        except FileNotFoundError:
          logger.warning(f"No .gitlab-ci.yml found for repo {repo.name} in branch {branch} under {path}. Skipping.")
          print(f"[red] No pipeline found for repo {repo.name} in branch {branch}. Skipping.[/red]")
          summary[repo.name][branch] = ":x:"
          progress.update(task, advance=1)
          continue
        pipeline = GitlabCIImporter().get_pipeline(file)
        file.close()

        """ Can be used to delete jobs, that should not be run for this branch.
        Might be necessary, as the dev branch might accidentally be triggered on main.
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
        logger.info(f"Deleting jobs {jobs_to_delete} from pipeline of {repo.name} and branch {branch}...")
        for job in jobs_to_delete:
          pipeline.delete_job(job)
        """

        new_dependencies, pipeline = changeToUpdatedImages(progress, docker_migration, pipeline)
        dependencies = dependencies.union(new_dependencies)
        # Convert the pipeline to Github Actions format, depending on the number of branches
        if len(branches_to_be_migrated[str(repoID)]) <= 1:
          pipelineConverter = GithubSubTreeConverter(architecture, pipeline, github_repo_prefix[repo.name],
                                                     compatibleImages=docker_migration.nativeImage, rebuild=rebuild, )
          convertedPipeline = pipelineConverter.parse_pipeline(repoID)
          file_path = os.path.join(file_path_base, repo.name + ".yml")
        else:
          pipelineConverter = GithubSubTreeConverter(architecture, pipeline,
                                                     github_repo_prefix[repo.name] + "/" + branch,
                                                     compatibleImages=docker_migration.nativeImage, rebuild=rebuild, )
          convertedPipeline = pipelineConverter.parse_pipeline(repoID)
          file_path = os.path.join(file_path_base, repo.name + "_" + branch + ".yml")
        print(f"[green]Converted pipeline of {repo.name} and branch {branch}")
        summary[repo.name][branch] = ":white_check_mark:"
        # Write and commit action
        writeStringToFile(file_path, convertedPipeline)
        subtree_repo.git.add(all=True)
        subtree_repo.index.commit(f"Migrated pipeline of {repo.name} and branch {branch} from Gitlab to Github")
        progress.update(task, advance=1)
  docker_migration.write_images_being_migrated()
  print()
  print("[bold]Summary of migration:")
  table = Table("Repository name", "Branch", "Migration successful")
  for repoID in config.repoIDS:
    repo = architecture.get_repo_by_ID(repoID)
    for branch in branches_to_be_migrated[repoID]:
      if branch in summary[repo.name]:
        table.add_row(repo.name, branch, summary[repo.name][branch])
      else:
        table.add_row(repo.name, branch, ":x:")
  console.print(table)
  if dependencies:
    print()
    print("[yellow]During the pipeline migration, dependencies with images from the following repos were detected:")
    for repo in dependencies:
      print(repo)
    print("[yellow]Please migrate them to the new registry as well, before running the actions.")


if __name__ == "__main__":
  gitlabFilePath = ".gitlab-ci.yml"
  githubFilePath = ".main.yml"
  GitlabToGithub(gitlabFilePath, "pipeline", ["GitlabToken", "URL", "ID"])
