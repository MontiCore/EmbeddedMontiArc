from typing_extensions import overload

from migrationTool.migration_types import Architecture
from migrationTool.pipeline_migration.GithubConverter import GithubActionConverter
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
    :param repoID: ID of the repository
    :return: String of the pipeline
    """
    return super().parse_pipeline(repoID, self.repoPath)

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
