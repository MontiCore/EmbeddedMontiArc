import re

from migrationTool.migration_types import Architecture
from migrationTool.pipelineMigration.GithubConverter import GithubActionConverter
from migrationTool.pipelineMigration.Job import Job


class GithubSubTreeConverter(GithubActionConverter):
  # Specialized converter for Monorepo variant"
  def __init__(self, architecture: Architecture, pipeline, repoPath, repoID, compatibleImages=set(),
               rebuild: bool = False, ):
    """
    Initializes the GithubSubTreeConverter class.
    :param pipeline: Pipeline object
    :param repoNames: IDs mapped to names of the repository
    :param repoPath: IDs mapped to paths to the repository
    """
    super().__init__(architecture, pipeline, compatibleImages=compatibleImages, rebuild=rebuild)
    self.repoPath = repoPath
    self.repoID = repoID

  # Check for correct overwriting attribuites
  def parse_pipeline(self, name: str, secrets: list[str]) -> str:
    """
        Parses the pipeline of a subtree Repo and returns it in the converted form as a string.
    :param name: Name of the pipeline
    :param secrets: Secrets to be used in the pipeline
    :return: String of the pipeline
    """
    self.file_change_job_needed = False
    pipelineString = ""
    pipelineString += f"name: {name}\n"
    pipelineString += "on:\n"
    pipelineString += "\tpush:\n"
    pipelineString += "\t\tpaths:\n"
    pipelineString += "\t\t\t- '" + self.repoPath + "/**'\n"
    pipelineString += "\tworkflow_dispatch:\n"
    pipelineString += "\tpull_request:\n"
    pipelineString += "env:\n"
    pipelineString += f"\tCI_PROJECT_ID : {self.repoID}\n"
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

  def parse_job(self, job: Job, secrets: list[str] = []) -> str:
    """
    Parses a job in a subtree repo pipeline and returns it in the converted form as a string.
    :param job: Job object
    :param secrets: List of secrets to be used in the job
    :return: Converted job
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
      # If the job has a script, add the repo path to the script
      job.script = [f"cd {self.repoPath}"] + job.script

    # Uses the normal githubaction converter to parse the job
    jobString = super().parse_job(job, secrets)

    prefix = f"{self.repoPath}/"

    def replace_paths_with_prefix(match):
      full_block = match.group(1)
      paths = match.group(2).splitlines()
      prefixed_paths = [(f"            {"- " + prefix if path.strip().startswith('- ') else prefix}"
                         f"{path.strip()[4:] if path.strip().startswith('- ./') else path.strip()}") for path in paths]
      full_block_without_path = re.sub(r"path: \|.*", "", full_block, flags=re.DOTALL)
      return (full_block_without_path + "path: |\n" + "\n".join(prefixed_paths) + "\n")

    # Add the prefix to the paths in the artifact download blocks
    artifactDownloadPattern = (r"(- name: .*\n\s+uses: actions/download-artifact@v4\n(?:\s+.+\n)*?\s+path: \|\n)(("
                               r"?:\s+.+\n?)+?)")
    jobString = re.sub(artifactDownloadPattern, replace_paths_with_prefix, jobString)

    # Add the prefix to the paths in the upload pages blocks
    uploadPagesPattern = r"(- name: Upload Pages\s+uses: actions/upload-pages-artifact@v3\s+with:\s+path: )(.+)"

    def add_prefix_to_upload_pages_path(match):
      full_block = match.group(1)  # Der Block bis einschließlich `path:`
      path_value = match.group(2).strip()  # Der ursprüngliche `path`-Wert
      return full_block + prefix + path_value  # Präfix hinzufügen

    jobString = re.sub(uploadPagesPattern, add_prefix_to_upload_pages_path, jobString)

    if job.trigger:
      triggered_repo = job.trigger["project"].split("/")[-1]
      repo = self.architecture.get_repo_by_name(triggered_repo)
      if repo:
        multiple_branches = True if len(repo.get_branches_to_be_migrated()) > 1 else False
      else:
        multiple_branches = False
      if multiple_branches:
        workloflow_name = triggered_repo + "_" + job.trigger["branch"] + ".yml"
      else:
        workloflow_name = triggered_repo + ".yml"

      # Regex für die Werte
      pattern = r"(WORKFLOW_FILE:\s+)(\{.*?\}\.yml)|(BRANCH:\s+)(\w+)|(REPO:\s+)(\w+)"

      # Ersetzen der Werte
      jobString = re.sub(pattern, lambda m: m.group(1) + f"{workloflow_name}" if m.group(1) else m.group(
        3) + "${{ github.ref_name }}" if m.group(3) else m.group(5) + "${{github.repository}}", jobString)
    return jobString
