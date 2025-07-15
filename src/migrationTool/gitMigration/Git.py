import logging
import os
import time
from datetime import datetime

import git
import typer
from rich import print
from rich.console import Console
from rich.progress import Progress
from rich.prompt import Confirm
from rich.table import Table
from tqdm import tqdm

logger = logging.getLogger(__name__)


class Git:
  def __init__(self):
    pass

  def reset_remote_origin(self, repo, new_remote_url):
    """
    Removes the remote origin and replaces it with a new one.
    :param repo: Repo object
    :param new_remote_url: New remote URL
    """
    try:
      # Remove remote origin if it exists
      self.remove_remote_origin(repo)

      # Add new remote origin
      repo.create_remote("origin", new_remote_url)
      logger.info(f"Remote 'origin' wurde erfolgreich auf '{new_remote_url}' gesetzt.")
    except Exception as e:
      logger.error(f"Fehler beim Zurücksetzen des Remote 'origin': {e}")

  def remove_remote_origin(self, repo):
    """
    Remove the remote origin from the repository.
    :param repo: Repo Object
    :return:
    """
    try:
      if "origin" in repo.remotes:
        origin = repo.remote(name="origin")
        # repo.delete_remote("origin")
        repo.delete_remote(origin.name)
        logging.info("Remote 'origin' wurde erfolgreich entfernt.")
        typer.echo("Remote 'origin' wurde erfolgreich entfernt.")
    except Exception as e:
      logger.error(f"Fehler beim Zurücksetzen des Remote 'origin': {e}")
      typer.echo("[red]Fehler beim Zurücksetzen des Remote 'origin'. PLease be carefull when manually pushing![/red]")

  def init_repo(self, repo_name):
    """
    Initialize a git repository in the given path and makes init commit.
    :param repo_name: Name of the repository
    """
    # Check if the repository already exists
    repo_path = os.path.join(os.getcwd(), "repos", repo_name)
    if os.path.exists(repo_path):
      logger.info(f"Repository '{repo_name}' already exists at '{repo_path}'.")
      print(f"Repository '{repo_name}' already exists at '{repo_path}'")
      return git.Repo(repo_path)
    else:
      logger.info(f"Creating new repository '{repo_name}' at '{repo_path}'.")
      print(f"Creating new repository '{repo_name}' at '{repo_path}'")
      repo = git.Repo.init(repo_path)
      # Create an empty README file
      readme_path = os.path.join(repo_path, "README.md")
      with open(readme_path, "w") as f:
        f.write("# " + repo_name)
      # Add the README file to the repository
      repo.git.add("README.md")
      # Commit the changes
      repo.index.commit("Initial commit")
      if "master" not in repo.branches:
        repo.create_head("master")
        repo.heads["master"].checkout()
      return repo

  def add_subtree(self, repo, subtree_repo_name, subtree_repo_path, prefix="", branch="master"):
    """
    Add a subtree to the repository.
    :param repo: Repository object
    :param subtree_repo_path: Path to the repository to be added as a subtree
    :param subtree_repo_name: Name of the repository to be added as a subtree
    :param prefix: Group prefix of the monorepo
    :param branch: Branch of the subtree to be added
    """
    # Check if the subtree already exists
    subtree_path = os.path.join(prefix, subtree_repo_name)
    if subtree_path in [item.path for item in repo.tree().traverse()]:
      logger.warning(f"Subtree '{subtree_repo_name}' already exists at '{subtree_path}'.")
      print(f"[red] Subtree '{subtree_repo_name}' already exists'.")
      return ":white_check_mark: Subtree already exists"
    if not os.path.exists(subtree_repo_path):
      logger.error(f"Subtree repository '{subtree_repo_name}' does not exist at '{subtree_repo_path}'.")
      print(f"[red] Subtree repository '{subtree_repo_name}' does not exist'.")
      return ":x: Subtree repository does not exist"
    # Creates new branch in which the subtree is added
    if subtree_repo_name in repo.remotes:
      repo.delete_remote(subtree_repo_name)
    repo.create_remote(subtree_repo_name, subtree_repo_path)
    repo.git.fetch(subtree_repo_name, branch)
    try:
      repo.git.subtree("add", "--prefix", subtree_path, subtree_repo_name, branch)
    except git.exc.GitCommandError as e:
      logger.error(f"Subtree could not be added at '{subtree_path}'." + str(e))
      return ":x: Subtree could not be added"
    logger.info(f"Branch {branch} added as a subtree at '{subtree_path}.")
    print(f"Branch {branch} added as a subtree.")
    return ":white_check_mark: Subtree added"

  def add_subtree_branch(self, repo, subtree_repo_name, subtree_repo_path, branch, prefix=""):
    """
    Adds (multiple) branches of a repo as subtree to the repository.
    :param name: Repository name
    :param subtree_repo_name: Name of the repository to be added as a subtree
    :param branch: Branch of the subtree to be added
    :param prefix: Prefix for the subtree path
    """

    # Check if the subtree already exists
    subtree_path = os.path.join(prefix, subtree_repo_name, branch)
    if subtree_path in [item.path for item in repo.tree().traverse()]:
      logger.warning(f"Subtree '{subtree_repo_name}' already exists at '{subtree_path}'.")
      print(f"[red] Subtree '{subtree_repo_name}' and branch {branch} already exists'.")
      return ":white_check_mark: Subtree already exists"
    if not os.path.exists(subtree_repo_path):
      logger.error(f"Subtree repository '{subtree_repo_name}' does not exist at '{subtree_repo_path}'.")
      print(f"[red] Subtree repository '{subtree_repo_name}' does not exist'.")
      return ":x: Subtree repository does not exist"
    if subtree_repo_name in repo.remotes:
      repo.delete_remote(subtree_repo_name)
    repo.create_remote(subtree_repo_name, subtree_repo_path)
    repo.git.fetch(subtree_repo_name, branch)

    try:
      repo.git.subtree("add", "--prefix", subtree_path, subtree_repo_name, branch)
    except git.exc.GitCommandError as e:
      logger.error(f"Subtree could not be added at '{subtree_path}'." + str(e))
      print(f"[red] Subtree could not be added at '{subtree_path}'.")
      return ":x: Subtree could not be added"
    logger.info(f"Branch {branch} added as a subtree at '{subtree_path}.")
    print(f"Branch {branch} added as a subtree.")
    return ":white_check_mark: Subtree added"

  def add_repos_as_subtree(self, target_repo_name, target_repo_namespace, architecture, repoIDs=[]):
    """
    Adds only master branch as subtree to the target repository.
    :param target_repo_name: Name of the target GitHub repository
    :param target_repo_namespace: Namespace of the target GitHub repository
    :param architecture: Architecture object containing the repositories to be added
    :param repoIDs: List of repository IDs to be added as subtrees. If empty, all repositories in the architecture
    are added.
    """
    console = Console()
    if not repoIDs:
      repoIDs = architecture.repoIDs

    target_repo = self.init_repo(target_repo_name)
    branch_name = "Migration_" + str(datetime.now().strftime("%Y-%m-%d_at_%H-%M-%S"))
    print(f"Creating branch {branch_name} in {target_repo_name}...")
    if branch_name in target_repo.branches:
      if Confirm.ask(f"[red]Branch {branch_name} already exists. Do you want to delete it?"):
        target_repo.delete_head(branch_name)
    target_repo.create_head(branch_name)
    target_repo.heads[branch_name].checkout()
    numberIterations = 0
    for repoID in repoIDs:
      repo = architecture.get_repo_by_ID(repoID)
      numberIterations += len(repo.get_branches_to_be_migrated())

    summary = {}
    with Progress(auto_refresh=True, refresh_per_second=0.5) as progress:
      task1 = progress.add_task("Adding subtrees", total=numberIterations)
      for repoID in repoIDs:
        summary[repoID] = {}
        repo = architecture.get_repo_by_ID(repoID)
        repo_namespace = "/".join([i for i in repo.namespace.split("/") if i not in target_repo_namespace])
        branches_to_be_migrated = repo.get_branches_to_be_migrated()
        multiple_branches = len(branches_to_be_migrated) > 1
        logger.info(f"Adding {repo.name} as a subtree to {repo_namespace}...")
        print("-------------------------------")
        print(f"Adding {repo.name} as a subtree")
        print()
        table = Table("Branch", "Path")
        for branch in branches_to_be_migrated:
          table.add_row(branch, (repo_namespace if not multiple_branches else repo_namespace + "/" + branch), )
        progress.stop()
        console.print(table)
        progress.start()
        for branch in branches_to_be_migrated:
          if multiple_branches:
            summary[repoID][branch] = self.add_subtree_branch(target_repo, repo.name, repo.path, prefix=repo_namespace,
                                                              branch=branch, )
          else:
            summary[repoID][branch] = self.add_subtree(target_repo, repo.name, repo.path, prefix=repo_namespace,
                                                       branch=branch, )
          progress.update(task1, advance=1)
    print()
    print("[bold]Summary:")
    for repoID in summary:
      repo = architecture.get_repo_by_ID(repoID)
      print(f"[bold]{repo.name}:[/bold]")
      table = Table("Branch", "Status")
      for branch in summary[repoID]:
        table.add_row(branch, summary[repoID][branch])
      console.print(table)
      print()

  def has_submodules(self, repo_path):
    """Check if the repo has a .gitmodules file"""
    return os.path.isfile(os.path.join(repo_path, ".gitmodules"))

  def get_submodule_paths(self, repo_path):
    """Extract submodule paths from .gitmodules"""
    submodule_paths = []
    gitmodules_path = os.path.join(repo_path, ".gitmodules")
    if os.path.isfile(gitmodules_path):
      with open(gitmodules_path, "r") as f:
        for line in f:
          if line.strip().startswith("path = "):
            submodule_paths.append(line.strip().split("= ")[1])
    return submodule_paths

  def absorb_submodules(self, repo_path):
    if not self.has_submodules(repo_path):
      logger.warning(f"Absorb submodules called for {repo_path} but no submodules found.")
      return True
    repo = git.Repo(repo_path)
    for submodule_path in self.get_submodule_paths(repo_path):
      logger.info(f"Absorbing submodule {submodule_path}")
      try:
        repo.git.fetch(submodule_path, "HEAD")
      except git.exc.GitCommandError as e:
        logger.error(f"Error fetching submodule {submodule_path}: {e}")
        return False
      gitmodules_file = os.path.join(repo_path, ".gitmodules")
      if os.path.exists(gitmodules_file):
        repo.git.rm(gitmodules_file)
      repo.git.rm(submodule_path)
      repo.git.commit("-m", "Prepare to absorb submodule " + submodule_path)
      repo.git.subtree("add", "--prefix", submodule_path, "FETCH_HEAD")
      logger.info(f"Finished absorbing submodule {submodule_path}")
    return True

  def checkout_branches(self, repo, absorb_submodules=False):
    """
    Checkout all branches available in the remote repository.
    :param repo: repository objec
    :param absorb_submodules: If True, absorb all submodules after checking out each branch
    """
    console = Console()
    table = Table("Branch", "Status")
    default_branch = repo.active_branch.name
    repo.remotes.origin.fetch()  # Fetch all branches from the remote
    for branch in tqdm(repo.remotes.origin.refs, desc="Checking out all branches"):  # Iterate over all remote branches
      branch_name = branch.name.split("/")[-1]  # Extract branch name
      if branch_name == "HEAD" or branch_name == default_branch:
        continue
      try:
        repo.git.checkout("-B", branch_name, branch.name)  # Create and checkout local branch tracking the remote
        logging.info(f"Checked out branch {branch_name}.")
        time.sleep(1)
        if absorb_submodules and self.has_submodules(repo.working_tree_dir):
          logger.info(f"Absorbing submodules {self.get_submodule_paths(repo.working_tree_dir)} in branch {branch_name}")
          repo.git.submodule("update", "--init", "--recursive")
          result = self.absorb_submodules(repo.working_tree_dir)
          if result:
            logger.info(f"Submodules absorbed for branch {branch_name}.")
            table.add_row(branch_name,
                          ":white_check_mark: [green] Successfully checked out; Submodules absorbed[/green]", )
          else:
            logger.info(f"Submodules absorption failed for branch {branch_name}.")
            table.add_row(branch_name,
                          ":heavy_exclamation_mark: [yellow] Successfully checked out; Error absorbing submodules ["
                          "/yellow]", )
        else:
          table.add_row(branch_name, ":white_check_mark: [green] Successfully checked out [/green]", )
      except Exception as e:
        logging.warning(f"Error checking out branch or submodule {branch_name}: {e}")
        table.add_row(branch_name, ":x: [red] Error checking out branch [/red]")
    repo.git.checkout(default_branch)
    if absorb_submodules and self.has_submodules(repo.working_tree_dir):
      logger.info(f"Absorbing submodule {self.get_submodule_paths(repo.working_tree_dir)} in branch {default_branch}")
      repo.git.submodule("update", "--init", "--recursive")
      result = self.absorb_submodules(repo.working_tree_dir)
      if result:
        logger.info(f"Submodules absorbed for branch {default_branch}.")
        table.add_row(default_branch,
                      ":white_check_mark: [green] Successfully checked out; Submodules absorbed[/green]", )
      else:
        logger.info(f"Submodules absorption failed for branch {default_branch}.")
        table.add_row(default_branch,
                      ":heavy_exclamation_mark: [yellow] Successfully checked out; Error absorbing submodules ["
                      "/yellow]", )
    else:
      table.add_row(default_branch, ":white_check_mark: [green] Successfully checked out [/green]", )
    console.print(table)
