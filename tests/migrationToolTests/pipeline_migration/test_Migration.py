import os
import shutil
from unittest import TestCase
from unittest.mock import MagicMock, patch

import git

from migrationTool.migration_types import Architecture, Config
from migrationTool.pipeline_migration import GitlabToGithubSubtree, GitlabToGithub


class Test(TestCase):
  def setUpMono(self):
    current_file_dir = os.path.dirname(__file__)
    testresources_path = os.path.abspath(os.path.join(current_file_dir, "../../testRessources/MigrationMono"))
    print(os.getcwd())
    shutil.copytree(testresources_path, os.path.join(os.getcwd(), "TEST"))
    self.architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "TEST", "architecture.yaml"))
    self.config = Config(os.path.join(os.getcwd(), "TEST", "config.yaml"))

    monorepo_path = os.path.join(os.getcwd(), "TEST", "repos", self.config.monorepoName)
    repo = git.Repo.init(monorepo_path)
    with open(os.path.join(monorepo_path, "temp.txt"), "w") as f:
      f.write("This is a temporary file to ensure the repo is not empty.")
    repo.git.add(all=True)
    repo.index.commit("Initial commit")
    os.chdir(os.path.join(os.getcwd(), "TEST"))

  def setUpRepo(self):
    current_file_dir = os.path.dirname(__file__)
    testresources_path = os.path.abspath(os.path.join(current_file_dir, "../../testRessources/MigrationRepo"))
    shutil.copytree(testresources_path, os.path.join(os.getcwd(), "TEST"))
    self.architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "TEST", "architecture.yaml"))
    self.config = Config(os.path.join(os.getcwd(), "TEST", "config.yaml"))

    monorepo_path = os.path.join(os.getcwd(), "TEST", "repos", "repoA")
    repo = git.Repo.init(monorepo_path)
    with open(os.path.join(monorepo_path, "temp.txt"), "w") as f:
      f.write("This is a temporary file to ensure the repo is not empty.")
    repo.git.add(all=True)
    repo.index.commit("Initial commit")
    os.chdir(os.path.join(os.getcwd(), "TEST"))

  def tearDown(self):
    path = os.path.abspath(os.path.join(os.getcwd(), ".."))
    os.chdir(path)
    shutil.rmtree(os.path.join(os.getcwd(), "TEST"))

  @patch("gitlab.Gitlab", autospec=True)
  @patch('rich.prompt.Confirm.ask')
  def test_gitlab_to_github_mono(self, mock_confirm, mock_gitlab):
    print(os.getcwd())
    self.setUpMono()
    mock_gitlab.return_value = MagicMock()
    mock_gitlab.return_value.auth = MagicMock(return_value=None)
    mock_confirm.side_effect = [False, False, False, False, True, False]
    GitlabToGithubSubtree(self.architecture, self.config)

    self.assertTrue(
      os.path.exists(os.path.join(os.getcwd(), "repos", self.config.monorepoName, ".github", "workflows", "repoA.yml")))
    self.assertTrue(
      os.path.exists(os.path.join(os.getcwd(), "repos", self.config.monorepoName, ".github", "workflows", "repoB.yml")))

  @patch("gitlab.Gitlab", autospec=True)
  @patch('rich.prompt.Confirm.ask')
  def test_gitlab_to_github_Repo(self, mock_confirm, mock_gitlab):
    self.setUpRepo()
    mock_gitlab.return_value = MagicMock()
    mock_gitlab.return_value.auth = MagicMock(return_value=None)
    mock_confirm.side_effect = [False, False, False, False, True, False]
    GitlabToGithub(self.architecture, self.config)
    print(os.getcwd())
    self.assertTrue(os.path.exists(os.path.join(os.getcwd(), "repos", "repoA", ".github", "workflows", "repoA.yml")))
