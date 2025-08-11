import re

from typing_extensions import overload

from migrationTool.migration_types import Architecture
from migrationTool.pipeline_migration.GithubConverter import GithubActionConverter
from migrationTool.pipeline_migration.Job import Job
from migrationTool.pipeline_migration.Pipeline import Pipeline


class GithubSubTreeConverter(GithubActionConverter):
  # Specialized converter for Monorepo variant"
  def __init__(self, architecture: Architecture, pipeline: Pipeline, repoPath: str, compatibleImages=set(),
               rebuild: bool = False, ):
    """
    Initializes the GithubSubTreeConverter class.
    :param pipeline: Pipeline object
    :param repoPath: Path to this repository inside the monorepo
    """
    super().__init__(architecture, pipeline, compatibleImages=compatibleImages, rebuild=rebuild)
    self.repoPath = repoPath
    # Adapt model
    self.__adapt_model()

  def parse_pipeline(self, repoID) -> str:
    """
    Parses the pipeline of a subtree Repo and returns it in the converted form as a string.
    :param name: Name of the pipeline
    :param secrets: Secrets to be used in the pipeline
    :return: String of the pipeline
    """
    repo = self.architecture.get_repo_by_ID(repoID)
    secrets = repo.secrets
    self.file_change_job_needed = False
    pipelineString = ""
    pipelineString += f"name: {repo.name}\n"
    pipelineString += "on:\n"
    pipelineString += "\tpush:\n"
    pipelineString += "\t\tpaths:\n"
    pipelineString += "\t\t\t- '" + self.repoPath + "/**'\n"
    pipelineString += "\tworkflow_dispatch:\n"
    pipelineString += "\tpull_request:\n"
    pipelineString += "env:\n"
    pipelineString += f"\tCI_PROJECT_ID : {repoID}\n"
    if secrets:
      for secret in secrets:
        if type(secret) == tuple:
          if secret[0] != "CI_PROJECT_ID":
            pipelineString += f"\t{secret[0]} : " + f"{secret[1]}\n"
        else:
          if secret != "CI_PROJECT_ID":
            pipelineString += (f"\t{secret} : " + "${{ secrets." + f"{secret}" + " }}\n")
    if self.pipeline.variables:
      for var_name, var_value in self.pipeline.variables.items():
        pipelineString += f"\t{var_name} : " + f"{var_value}\n"

    pipelineString += "jobs:\n"
    for _, job in self.pipeline.jobs.items():
      # Check whether a job is only run if a file changes
      file_changes = (job.only and type(job.only) == dict and "changes" in job.only) or (
          job.exc and type(job.exc) == dict and "changes" in job.exc)
      if job.rules:
        for rule in job.rules:
          if "changes" in rule:
            file_changes = True
            break
      if file_changes:
        # If yes, add job to check
        self.file_change_job_needed = True
        pipelineString += self.create_file_change_job()
        break

    # Create jobs for the stages
    pipelineString += self.create_stage_jobs()
    # Parse all the normal jobs
    for job in self.pipeline.jobs:
      pipelineString += self.parse_job(self.pipeline.jobs[job], secrets)
      pipelineString += "\n"
    return self.set_indentation_to_two_spaces(pipelineString)

  def __adapt_model(self):
    """
    Adapts the model of the pipeline to the monorepo structure.
    :return:
    """

    def parse_path(path: str) -> str:
      """
      Parses a path and adds the repo path to it.
      :param path: Path from normal job
      :return: Path for monorepo job
      """
      path = path.split("/")
      if path[0] == ".":
        # If the path is relative, add the repo path to it
        return f"{self.repoPath}/{'/'.join(path[1:])}"
      else:
        return f"{self.repoPath}/{'/'.join(path)}"

    for job_name, job in self.pipeline.jobs.items():
      if job.artifacts:
        if "paths" in job.artifacts:
          # If the job has artifacts, add the repo path to the paths
          for i, path in enumerate(job.artifacts["paths"]):
            job.artifacts["paths"][i] = parse_path(path)
        elif "reports" in job.artifacts:
          # If the job has reports, add the repo path to the paths
          if type(job.artifacts["reports"]) == list:
            # If the report is a list, parse each path in the list
            for i, report in enumerate(job.artifacts["reports"]):
              if report.startswith("junit"):
                job.artifacts["reports"][i] = "junit:" + parse_path(report.split(":")[1])
          elif type(job.artifacts["reports"]) == str:
            # If the report is a single path, parse it
            job.artifacts["reports"] = parse_path(job.artifacts["reports"])
      if job.script:
        # If the job has a script, add the cd repo path to the script
        job.script = [f"cd {self.repoPath}"] + job.script

      if job.trigger:
        # If the job has a trigger, change to trigger the workflow in the monorepo
        triggered_repo = job.trigger["project"].split("/")[-1]
        repo = self.architecture.get_repo_by_name(triggered_repo)
        if repo:
          multiple_branches = True if len(repo.get_branches_to_be_migrated()) > 1 else False
        else:
          multiple_branches = False
        if multiple_branches:
          workloflow_name = triggered_repo + "_" + job.trigger["branch"] if "branch" in job.trigger else "" + ".yml"
        else:
          workloflow_name = triggered_repo + ".yml"
        job.trigger.pop("project")
        job.trigger["include"] = workloflow_name
