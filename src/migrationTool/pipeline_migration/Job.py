class Job:
  """
  A class representing a single job in a CI/CD pipeline.
  """

  def __init__(self, name: str, image: str, stage: str, script: list[str], needs: list[str] = [], exc: list[str] = [],
               artifacts={}, only: list[str] = [], allowFailure=False, rules=[], trigger=dict, variables=dict):
    """
    Initializes the job with the given parameters.
    :param name: Name of the job
    :param image: Docker image to be used for the job
    :param stage: Stage of the job in the pipeline
    :param script: Script to be executed in the job
    :param needs: List of jobs that need to be successful before this job can start
    :param exc: List of branches that should not trigger the job
    :param artifacts: Defines as dictionary with key 'paths' for the paths to be saved as artifacts. 'when' can be
    used to determine when the artifact should be saved. Possible values are 'on_success', and 'always'. Furthermore
    'expire in' can be used to set the expiration date in days. Finally 'reports', can be used to show test result
    files, which should be summarized.
    :param only: List of branches that should trigger the job
    :param allowFailure: Flag indicating if the job can fail without failing the pipeline
    :param rules: Rules for when the job should run. Is defined as dictionary with key 'if' for the bool expression
    and 'changes' for the file changes side condition
    :param trigger: Trigger other pipelines. Defined as dictionary with key 'include' a workflow in the same repo and
    'project' for a workflow in another repo. The latter can also use 'branch' to specify the branch to trigger.
    :param variables: Variables to be used in the job. Defines as (Name, Value) tuples
    :return: None
    """

    self.name = name
    self.image = image
    self.stage = stage
    self.script = script
    self.needs = needs
    self.exc = exc
    self.artifacts = artifacts
    self.only = only
    self.allowFailure = allowFailure
    self.rules = rules
    self.trigger = trigger
    self.variables = variables

  def __str__(self):
    result = f"Name: {self.name}\n"
    if self.image:
      result += f"Image: {self.image}\n"
    if self.stage:
      result += f"Stage: {self.stage}\n"
    if self.script:
      result += f"Script: {self.script}\n"
    if self.needs:
      result += f"Needs: {self.needs}\n"
    if self.exc:
      result += f"Except: {self.exc}\n"
    if self.artifacts:
      result += f"Artifacts: {self.artifacts}\n"
    if self.only:
      result += f"Only: {self.only}\n"
    if self.allowFailure:
      result += f"Allow Failure: {self.allowFailure}\n"
    if self.rules:
      result += f"Rules: {self.rules}\n"
    if self.trigger:
      result += f"Trigger: {self.trigger}\n"
    return result
