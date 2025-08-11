import os
import shutil

from git.repo.base import Repo as GitRepo
from unittest import TestCase

from migrationTool.migration_types import Repo
from migrationTool.git_migration.Git import Git


class TestRepo(TestCase):
  def test_get_repo(self):
    git = Git()
    git.init_repo("testRepo")

    repo = Repo("testRepo", "1", [], os.path.join(os.getcwd(), "repos", "testRepo"), "abc", ["master"], [], [])

    # Test
    getRepo = repo.get_repo()

    # Evaluate
    self.assertIsInstance(getRepo, GitRepo)

    # Clean up
    shutil.rmtree(os.path.join(os.getcwd(), "repos"))

  def test_read_from_architecture(self):
    architecture_dict = {'repoA': {'ID': '1', 'Namespace': 'test', 'Path': 'repos/repoA',
                                   'Secrets': {"Token": {"Secret": "Y", "Value": "1234"},
                                               "Password": {"Secret": "E", "Value": "ABC"}}, 'Branches': ['master'],
                                   'StaleBranches': ['docu'], 'DockerImages': ['ubuntu', 'windows10']},
                         'repoB': {'ID': '2', 'Namespace': 'test/src', 'Path': 'repos/repoB',
                                   'Branches': ['master', 'development'],
                                   'DockerImages': ['windows11', 'macOS', 'linux']}}

    for repoName in architecture_dict.keys():
      repoID, repo = Repo.read_from_Architecture(repoName, architecture_dict[repoName])
      self.assertIsInstance(repo, Repo, "Should return a Repo instance")
      self.assertIn(repo.name, architecture_dict.keys(), "Repo name should be in architecture dict")
      self.assertEqual(repo.ID, architecture_dict[repo.name]['ID'], "Repo ID should match architecture dict")
      self.assertEqual(repoID, architecture_dict[repo.name]['ID'], "Repo ID should match architecture dict")
      self.assertEqual(repo.namespace, architecture_dict[repo.name]['Namespace'],
                       "Repo namespace should match architecture dict")
      self.assertEqual(repo.path, os.path.join(os.getcwd(), architecture_dict[repo.name]['Path']),
                       "Repo path should match the full local one")
      self.assertEqual(repo.active_branches, architecture_dict[repo.name]['Branches'],
                       "Repo active branches should match architecture dict")
      if 'StaleBranches' in architecture_dict[repo.name]:
        self.assertEqual(repo.stale_branches, architecture_dict[repo.name]['StaleBranches'],
                         "Repo stale branches should match architecture dict")
      self.assertEqual(repo.images, architecture_dict[repo.name]['DockerImages'],
                       "Repo docker images should match architecture dict")
      for secret in repo.secrets:
        if type(secret) == tuple:
          self.assertEqual(secret[1],
                           "${{ secrets." + architecture_dict[repo.name]["Secrets"][secret[0]]["Value"] + " }}",
                           "Secret value should be a GitHub secret reference")
        elif type(secret) == str:
          self.assertTrue(secret in architecture_dict[repo.name]["Secrets"],
                          "Secret is not defined in architecture dict")
          self.assertIn((secret, architecture_dict[repo.name]["Secrets"][secret]["Value"]), repo.secrets_to_create,
                        "Secret should be in secrets to create")

  def test_get_branches_to_be_migrated(self):
    repo = Repo("testRepo", "1", [], os.path.join(os.getcwd(), "repos", "testRepo"), "abc", ["master"],
                ["development", "test"], [])

    branches = repo.get_branches_to_be_migrated()
    self.assertEqual(set(branches), {"development", "test", "master"}, "Should return all branches for migration")

  def test_as_yaml(self):
    architecture_dict = {'repoA': {'ID': '1', 'Namespace': 'test', 'Path': 'repos/repoA',
                                   'Secrets': {"Token": {"Secret": "Y", "Value": "1234"},
                                               "Password": {"Secret": "E", "Value": "ABC"}}, 'Branches': ['master'],
                                   'StaleBranches': ['docu'], 'DockerImages': ['ubuntu', 'windows10']},
                         'repoB': {'ID': '2', 'Namespace': 'test/src', 'Path': 'repos/repoB',
                                   'Branches': ['master', 'development'],
                                   'DockerImages': ['windows11', 'macOS', 'linux']}}

    for repoName in architecture_dict.keys():
      repoID, repo = Repo.read_from_Architecture(repoName, architecture_dict[repoName])

      yaml_data = repo.as_yaml()

      # Evaluate
      smallDict = {repoName: architecture_dict[repoName]}
      smallDict[repoName]['Path'] = os.path.join(os.getcwd(), smallDict[repoName]['Path'])
      secretList = []
      for secret in repo.secrets:
        if type(secret) == tuple:
          if secret[0] in smallDict[repoName]['Secrets']:
            secretList.append((secret[0], secret[1]))
          else:
            self.fail(f"Secret {secret[0]} not found in architecture dict")
        elif type(secret) == str:
          if secret in smallDict[repoName]['Secrets']:
            secretList.append(secret)
          else:
            self.fail(f"Secret {secret} not found in architecture dict")
      smallDict[repoName]['Secrets'] = secretList
      self.assertIsInstance(yaml_data, dict, "Should return a dict")
      self.assertDictEqual(yaml_data, smallDict, "Yaml data should match architecture dict")
