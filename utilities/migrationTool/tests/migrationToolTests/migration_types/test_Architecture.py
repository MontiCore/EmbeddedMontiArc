import os
from unittest import TestCase
import yaml

from migrationTool.migration_types import Architecture, Repo


class TestArchitcture(TestCase):
  def test_add_repo(self):
    architecture = Architecture("mock_architecture")
    self.assertEqual(architecture.repos, {}, "Architecture repos should be empty at start")
    repositoryA = Repo("repoA", "1", [""], "repos", "test", ["master", "development"], [], [])

    # Test
    architecture.add_repo(repositoryA)

    # Evaluate
    self.assertEqual(architecture.repos["1"], repositoryA, "Repository should be added to architecture")
    self.assertEqual(len(architecture.repos), 1, "Wrong number of repositories in architecture")
    self.assertIsInstance(architecture.get_repo_by_ID("1"), Repo, "Should be a Repo instance")

  def test_dump_yaml(self):
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "architecture.yaml")),
                     "File should not exist before test")

    architecture = Architecture(os.path.join(os.getcwd(), "architecture.yaml"))
    repositoryA = Repo("repoA", "1", ["ubuntu", "windows10"], "repos/repoA", "test", ["master"], ["docu"], ["token"],
                       [("test", "value")])
    repositoryB = Repo("repoB", "2", ["windows11", "macOS", "linux"], "repos/reopB", "test/src",
                       ["master", "development"], [], [])
    architecture.add_repo(repositoryA)
    architecture.add_repo(repositoryB)

    # Test
    architecture.dump_yaml(verbose=True)

    # Evaluate
    with open(os.path.join(os.getcwd(), "architecture.yaml"), "r") as file:
      file_content = yaml.safe_load(file)

    architecture_dict = {
      'repoA': {'ID': '1', 'Namespace': 'test', 'Path': 'repos/repoA', 'Secrets': ['token'], 'Branches': ['master'],
                'StaleBranches': ['docu'], 'DockerImages': ['ubuntu', 'windows10']},
      'repoB': {'ID': '2', 'Namespace': 'test/src', 'Path': 'repos/reopB', 'Secrets': [],
                'Branches': ['master', 'development'], 'DockerImages': ['windows11', 'macOS', 'linux']}}
    self.assertDictEqual(file_content, architecture_dict)

    # Test adding to this file
    repositoryC = Repo("repoC", "3", ["windows11", "macOS", "linux"], "repos/reopC", "test/src",
                       ["master", "development"], [], [])
    architecture = Architecture(os.path.join(os.getcwd(), "architecture.yaml"))
    architecture.add_repo(repositoryC)
    architecture.add_repo(repositoryB)  # Adding an existing repo should not change it
    architecture.dump_yaml(verbose=True)

    # Evaluate
    with open(os.path.join(os.getcwd(), "architecture.yaml"), "r") as file:
      file_content = yaml.safe_load(file)

    architecture_dict["repoC"] = {'ID': '3', 'Namespace': 'test/src', 'Path': 'repos/reopC', 'Secrets': [],
                                  'Branches': ['master', 'development'],
                                  'DockerImages': ['windows11', 'macOS', 'linux']}
    self.assertDictEqual(file_content, architecture_dict)

    # Clean up
    os.remove(os.path.join(os.getcwd(), "architecture.yaml"))
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "architecture.yaml")),
                     "File should be deleted after test")

  def test_load_architecture(self):
    architecture_dict = {'repoA': {'ID': '1', 'Namespace': 'test', 'Path': 'repos/repoA',
                                   'Secrets': {"Token": {"Secret": "Y", "Value": "1234"},
                                               "Password": {"Secret": "E", "Value": "ABC"}}, 'Branches': ['master'],
                                   'StaleBranches': ['docu'], 'DockerImages': ['ubuntu', 'windows10']},
                         'repoB': {'ID': '2', 'Namespace': 'test/src', 'Path': 'repos/repoB',
                                   'Branches': ['master', 'development'],
                                   'DockerImages': ['windows11', 'macOS', 'linux']}}
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "architecture.yaml")),
                     "File should not exist before creation")
    with open(os.path.join(os.getcwd(), "architecture.yaml"), "w") as file:
      yaml.dump(architecture_dict, file)
    self.assertTrue(os.path.exists(os.path.join(os.getcwd(), "architecture.yaml")), "File should exist")

    # Test
    architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "architecture.yaml"))

    # Evaluate
    self.assertIsInstance(architecture, Architecture, "Should return an Architecture instance")
    self.assertEqual(len(architecture.repos), 2, "Wrong number of repositories in architecture")
    for repoID in architecture.repoIDs:
      repo = architecture.get_repo_by_ID(repoID)
      self.assertIsInstance(repo, Repo, "Should return a Repo instance")
      self.assertIn(repo.name, architecture_dict.keys(), "Repo name should be in architecture dict")
      self.assertEqual(repo.ID, architecture_dict[repo.name]['ID'], "Repo ID should match architecture dict")
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

    # Clean up
    os.remove(os.path.join(os.getcwd(), "architecture.yaml"))
    self.assertFalse(os.path.exists(os.path.join(os.getcwd(), "architecture.yaml")),
                     "File should be deleted after test")

  def test_get_repo_by_id(self):
    architecture = Architecture("mock_architecture")
    self.assertEqual(architecture.repos, {}, "Architecture repos should be empty at start")
    repositoryA = Repo("repoA", "1", [""], "repos", "test", ["master", "development"], [], [])

    # Test
    architecture.add_repo(repositoryA)

    # Evaluate
    self.assertEqual(architecture.get_repo_by_ID("1"), repositoryA, "Repository should be added to architecture")
    self.assertEqual(len(architecture.repos), 1, "Wrong number of repositories in architecture")
    self.assertIsInstance(architecture.get_repo_by_ID("1"), Repo, "Should be a Repo instance")

  def test_get_repo_by_name(self):
    architecture = Architecture("mock_architecture")
    self.assertEqual(architecture.repos, {}, "Architecture repos should be empty at start")
    repositoryA = Repo("repoA", "1", [""], "repos", "test", ["master", "development"], [], [])

    # Test
    architecture.add_repo(repositoryA)

    # Evaluate
    self.assertEqual(architecture.get_repo_by_name("repoA"), repositoryA, "Repository should be added to architecture")
    self.assertEqual(len(architecture.repos), 1, "Wrong number of repositories in architecture")
    self.assertIsInstance(architecture.get_repo_by_name("repoA"), Repo, "Should be a Repo instance")

  def test_get_docker_images(self):
    architecture = Architecture("mock_architecture")
    self.assertEqual(architecture.repos, {}, "Architecture repos should be empty at start")
    repositoryA = Repo("repoA", "1", ["ubuntu", "windows10"], "repos", "test", ["master", "development"], [], [])
    repositoryB = Repo("repoB", "2", ["windows11", "macOS", "linux"], "repos", "test", ["master", "development"], [],
                       [])
    architecture.add_repo(repositoryA)
    architecture.add_repo(repositoryB)

    # Test
    images = architecture.get_docker_images()

    # Evaluate
    self.assertEqual(architecture.get_repo_by_name("repoA"), repositoryA, "Repository should be added to architecture")
    self.assertEqual(len(architecture.repos), 2, "Wrong number of repositories in architecture")
    self.assertDictEqual(images, {"1": ["ubuntu", "windows10"], "2": ["windows11", "macOS", "linux"]},
                         "Should return all docker images from all repos")
