import os

import yaml
from rich import print


class Config:
  """
  One of the main data classes of the migration tool. Used to store the configuration for the migration
  """

  def __init__(self, path="config.yaml"):
    with open(path, "r") as file:
      data = yaml.safe_load(file)
    self.url = data["URL"]
    self.sourceToken = data["SourceToken"]
    self.sourceUser = data["SourceUser"]
    self.targetToken = data["TargetToken"]
    self.targetUser = data["TargetRepoOwner"]
    self.repoIDS = [str(i) for i in data["Repos"]]
    self.monorepoName = data["MonorepoName"]
    self.monorepoNamespace = data["MonorepoNamespace"]

  @staticmethod
  def create_config_file(path: str = "config.yaml"):
    """
    Creates an empty configuration file for the Migration Tool
    :param path: Path to the configuration file
    """
    if not os.path.exists(path):
      data = {"URL": "Please add the URL of the source Git instance",
              "SourceToken": "Please add the private token of the source GitLab instance",
              "TargetToken:": "Please add the private token for GitHub",
              "Repos": "Please add the repo IDs for migration", "MonorepoName": "Please add the name of the monorepo",
              "SourceUser": "Please add the name of the user on the GitLab system",
              "TargetRepoOwner": "Please add the name of the user on GitHub",
              "MonorepoNamespace": "Please add the namespace of the monorepo, this removed from the beginning of the "
                                   "repo namespaces", "MonorepoName": "Please add the name of the monorepo "}
      with open(path, "w") as file:
        yaml.safe_dump(data, file)
      print(f"[green]Config file created at {path}[/green]")
      exit(0)
    else:
      print(f"[red]Config file already exists at {path}[/red]")
      exit(1)
