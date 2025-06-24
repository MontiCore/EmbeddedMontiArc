from unittest import TestCase

import yaml
import os

from migrationTool.migration_types import Config


class TestConfig(TestCase):
  def test_config(self):
    """
    Test reading a configuration file and checking if the attributes are set correctly. This should be done once an
    object is created.
    :return:
    """
    data = {"URL": "https://git.rwth-aachen.de/", "SourceToken": "secret", "TargetToken": "secret",
            "Repos": ["123", "456", "789"], "SourceUser": "David.Blum", "TargetRepoOwner": "DavidBlm",
            "MonorepoName": "dry-run2", "MonorepoNamespace": "monticore/EmbeddedMontiArc"}
    with open("config.yaml", "w") as outfile:
      yaml.safe_dump(data, outfile)

    # Test
    config = Config("config.yaml")

    # Evaluate
    self.assertIsInstance(config, Config, "Should be an instance of Config")
    self.assertEqual(config.url, data["URL"], "URL should match the one in the config file")
    self.assertEqual(config.sourceToken, data["SourceToken"], "Source token should match the one in the config file")
    self.assertEqual(config.targetToken, data["TargetToken"], "Target token should match the one in the config file")
    self.assertEqual(config.sourceUser, data["SourceUser"], "Source user should match the one in the config file")
    self.assertEqual(config.targetUser, data["TargetRepoOwner"], "Target user should match the one in the config file")
    self.assertEqual(config.repoIDS, data["Repos"], "Repo IDs should match the ones in the config file")
    self.assertEqual(config.monorepoName, data["MonorepoName"], "Monorepo name should match the one in the config file")
    self.assertEqual(config.monorepoNamespace, data["MonorepoNamespace"],
                     "Monorepo namespace should match the one in the config file")

    # Clean up
    os.remove("config.yaml")  # Remove the test config file after the test

  def test_create_config_file(self):
    """
    Test creating a configuration file. This should create a file with the default values.
    :return:
    """
    with self.assertRaises(SystemExit) as cm:
      Config.create_config_file("test_config.yaml")
    self.assertEqual(cm.exception.code, 0, "Exit code should be 0")

    # Evaluate
    self.assertTrue(os.path.exists("test_config.yaml"), "Config file should be created")

    # An existing config file should not be overwritten
    with self.assertRaises(SystemExit) as cm:
      Config.create_config_file("test_config.yaml")
    self.assertEqual(cm.exception.code, 1, "Exit code should be 1")

    # Clean up
    os.remove("test_config.yaml")
