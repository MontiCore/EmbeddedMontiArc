import os
import shutil
from unittest import TestCase
from unittest.mock import MagicMock, patch

import git

from migrationTool.migration_types import Architecture, Config
from migrationTool.pipelineMigration import GitlabToGithubSubtree


class Test(TestCase):
  def setUp(self):
    path = os.getcwd()
    path = path.split(os.path.sep)
    for i in range(len(path)):
      if path[i] == "tests":
        path = os.path.sep.join(path[:i + 1])
        break
    shutil.copytree(os.path.join(path, "testRessources", "Migration"), os.path.join(os.getcwd(), "TEST"))
    self.architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "TEST", "architecture.yaml"))
    self.config = Config(os.path.join(os.getcwd(), "TEST", "config.yaml"))

    monorepo_path = os.path.join(os.getcwd(), "TEST", "repos", self.config.monorepoName)
    repo = git.Repo.init(monorepo_path)
    with open(os.path.join(monorepo_path, "temp.txt"), "w") as f:
      f.write("This is a temporary file to ensure the repo is not empty.")
    repo.git.add(all=True)
    repo.index.commit("Initial commit")
    os.chdir(os.path.join(os.getcwd(), "TEST"))

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd()))

  @patch("gitlab.Gitlab", autospec=True)
  @patch('rich.prompt.Confirm.ask')
  def test_gitlab_to_github(self, mock_confirm, mock_gitlab):
    mock_gitlab.return_value = MagicMock()
    mock_gitlab.return_value.auth = MagicMock(return_value=None)
    mock_confirm.side_effect = [False, False, False, False, True, False]
    GitlabToGithubSubtree(self.architecture, self.config)

    self.assertTrue(
      os.path.exists(os.path.join(os.getcwd(), "repos", self.config.monorepoName, ".github", "workflows", "repoA.yml")))
    self.assertTrue(
      os.path.exists(os.path.join(os.getcwd(), "repos", self.config.monorepoName, ".github", "workflows", "repoB.yml")))
