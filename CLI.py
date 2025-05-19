import logging
import subprocess

import git
import typer
from rich import print
from rich.progress import Progress
from rich.progress import track
from rich.console import Console
from rich.table import Table

from src import Config, Architecture
from src.gitMigration.GitlabDownloader import GitlabDownloader
from src.Architecture import Architecture
from src.gitMigration.largeFiles import run_git_filter_repo
from src.gitMigration.repoCleaning import remove_lfs, split_large_files, remove_lfs_from_gitattributes
from src.gitMigration.Git import Git
from src.pipelineMigration import GitlabToGithubSubtree
from src.gitMigration import GithubUploader

app = typer.Typer(
    help="Migration tool for Git repositories. It from GitLab to GitHub. Curently only supports GitHub as target and migration as a monorepo. However more feature (like repo-wise migration) will be added in the future.")

# Logger konfigurieren, only log to file
file_handler = logging.FileHandler("output.log")
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.handlers = []  # Remove if you want to see logs in console
logger.addHandler(file_handler)


@app.command()
def create_config(output: str = typer.Option("config.yaml", help="Path for the config file")):
    """Create a config file."""
    if not output.endswith(".yaml"):
        output += ".yaml"
    Config.Config.create_config_file(output)


@app.command()
def scan_clone(config_path: str = typer.Option("config.yaml", help="Config file path"),
               architecture: str = typer.Option("architecture.yaml", help="Scan file path"),
               verbose: bool = typer.Option(False, help="Whether to show verbose output")):
    """Scan and clone repositories."""
    logger.info(f"Starting scan and clone using {config_path}")
    config = Config.Config(config_path)
    print(f"Config at {config_path} loaded")
    print()
    gitlab_downloader = GitlabDownloader(config)
    gitlab_downloader.clone()
    print()
    gitlab_downloader.scan(architecture, verbose)


@app.command()
def migrate_repos(architecture_path: str = typer.Option("architecture.yaml", help="Scan file path"),
                  remove_lfs_flag: bool = typer.Option(True, help="Whether to remove git LFS"),
                  remove_large_files_flag: bool = typer.Option(True,
                                                               help="Whether to delete large files and clean them from the history."),
                  split_large_files_flag: bool = typer.Option(False,
                                                              help="Whether to split large files into smaller chunks. Only compatible with bash on Linux and MacOS.")):
    """Migrate repositories (remove large files, etc)."""
    architecture = Architecture.load_architecture(architecture_path)
    logger.info("Scan file loaded at " + architecture_path)
    print("Scan file loaded at " + architecture_path)

    console = Console()
    table = Table("Repository name", "Path", "Remove LFS", "Remove large files", "Split large files")
    for repoID in architecture.repoIDs:
        repo = architecture.get_repo_by_ID(repoID)
        table.add_row(repo.name, repo.path, ":white_check_mark:" if remove_lfs_flag else ":x:",
                      ":white_check_mark:" if remove_large_files_flag else ":x:",
                      ":white_check_mark:" if split_large_files_flag else ":x:")
    console.print(table)

    numberIterations = 0
    for repoID in architecture.repoIDs:
        numberIterations += len(architecture.get_repo_by_ID(repoID).get_branches_to_be_migrated())
    with Progress(auto_refresh=True, refresh_per_second=0.5) as progress:
        task1 = progress.add_task("Migrating", total=numberIterations)
        summary = {}
        for repoID in architecture.repoIDs:
            repo = architecture.get_repo_by_ID(repoID)
            summary[repo.name] = True
            git_repo = git.Repo(repo.path)
            print()
            print("--------------------")
            print(f"[bold]Processing repo {repo.name} ")
            if remove_lfs_flag:
                # Check if the repo uses LFS

                lfs_check = subprocess.run(["git", "lfs", "ls-files"], cwd=repo.path,
                                           capture_output=True,
                                           text=True)
                # If yes remove LFS
                if lfs_check.stdout:
                    print("LFS detected, removing LFS")
                    remove_lfs(repo.path)

            for branch in repo.get_branches_to_be_migrated():
                try:
                    git_repo.git.checkout(branch)
                except git.exc.GitCommandError:
                    print(f"[red] Branch {branch} could not be checked out, skipping")
                    logger.error(f"Branch {branch} could not be checked out in repo {repo.name}")
                    summary[repo.name] = False

                if split_large_files:
                    split_large_files(repo.path)

                if remove_lfs_flag:
                    remove_lfs_from_gitattributes(repo.path)
                progress.update(task1, advance=1)
                print(f"Processed branch {branch}")

            if "master" in git_repo.branches:
                git_repo.git.checkout("master")
            elif "main" in git_repo.branches:
                git_repo.git.checkout("main")

    if remove_large_files_flag:
        for repoID in track(architecture.repoIDs, description="Cleaning large files, this may take a while"):
            repo_path = architecture.get_repo_by_ID(repoID).path
            run_git_filter_repo(repo_path)

    # Print summary
    table = Table("Repository name", "Migration successful")
    for repoID in architecture.repoIDs:
        repo = architecture.get_repo_by_ID(repoID)
        table.add_row(repo.name, ":white_check_mark:" if summary[repo.name] else ":x:")
    console.print(table)


@app.command()
def create_monorepo(config_path: str = typer.Option("config.yaml", help="Config file path"),
                    architecture_path: str = typer.Option("architecture.yaml", help="Scan file path")):
    """Create a large monorepo from repos."""
    logger.info(f"Creating monorepo using {config_path} and scan file {architecture_path}")
    config = Config.Config("config.yaml")
    architecture = Architecture.load_architecture("architecture.yaml")
    print(f"Config and scan file loaded")
    git = Git()

    # Build monorepo from cleaned repos
    git.add_repos_as_subtree(config.monorepoName, config.monorepoNamespace, architecture)


@app.command()
def convert_gh_actions(config_path: str = typer.Option("config.yaml", help="Config file path"),
                       architecture_path: str = typer.Option("architecture.yaml", help="Scan file path"),
                       rebuild_large_files: bool = typer.Option(False,
                                                                help="Whether to rebuild large files that have been split")):
    """Convert CI to GitHub Actions."""
    logger.info(f"Converting to GitHub Actions using {config_path} and scan file {architecture_path}")

    config = Config.Config(config_path)
    logger.info("Config loaded")
    architecture = Architecture.load_architecture(architecture_path)
    logger.info("Scan file loaded")

    # Convert pipelines
    GitlabToGithubSubtree(architecture.repoIDs, architecture, config, rebuild=rebuild_large_files)


@app.command()
def upload(config_path: str = typer.Option("config.yaml", help="Config file path"),
           architecture_path: str = typer.Option("architecture.yaml", help="Scan file path"),
           migrate_docker_images: bool = typer.Option(False,
                                                      help="Whether to move docker images to the monorepo"),
           disable_scanning: bool = typer.Option(False,
                                                 help="Whether to move docker images to the monorepo")):
    """Upload to GitHub."""
    config = Config.Config(config_path)
    logger.info("Config loaded")
    architecture = Architecture.load_architecture(architecture_path)
    logger.info("Scan file loaded")
    print(f"Config and scan file loaded")

    Uploader = GithubUploader.GithubUploader(config, architecture)
    if migrate_docker_images:
        Uploader.docker_image_migration_monorepo()
        print("Image migration workflow added")
    Uploader.upload_mono_repo(disable_scanning)


if __name__ == "__main__":
    app()
