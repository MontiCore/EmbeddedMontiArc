import os
from unittest import TestCase

from migrationTool.gitMigration.mavenSecrets import find_env_vars_in_repo


class Test(TestCase):
  def setUp(self):
    settings_content = """<?xml version="1.0" encoding="UTF-8"?>
        <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 
                  http://maven.apache.org/xsd/settings-1.0.0.xsd">
            <servers>
                <server>
                    <id>se-nexus</id>
                    <username>cibuild</username>
                    <password>${env.cibuild}</password>
                </server>
            </servers>
        </settings>
        """
    filename = "test_settings.xml"
    with open("settings.xml", "w") as f:
      f.write(settings_content)
    settings_content = """<?xml version="1.0" encoding="UTF-8"?>
        <settings xmlns="http://maven.apache.org/SETTINGS/1.0.0"
                  xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
                  xsi:schemaLocation="http://maven.apache.org/SETTINGS/1.0.0 
                  http://maven.apache.org/xsd/settings-1.0.0.xsd">
            <servers>
                <server>
                    <id>se-nexus</id>
                    <username>cibuild</username>
                    <password>${env.cibuild}</password>
                </server>
            </servers>
            <servers>
                <server>
                    <id>se-nexus</id>
                    <username>cibuild</username>
                    <password>${env.password}</password>
                </server>
            </servers>
        </settings>
        """
    with open("pom.xml", "w") as f:
      f.write(settings_content)

  def tearDown(self):
    os.remove("settings.xml")
    os.remove("pom.xml")
    self.assertFalse(os.path.exists("settings.xml"))
    self.assertFalse(os.path.exists("pom.xml"))

  def test_find_env_vars_in_repo(self):
    vars = None

    # Test
    vars = find_env_vars_in_repo(os.getcwd())

    # Evaluate
    self.assertIsNotNone(vars)
    self.assertIn("password", vars, "Password variable not detected")
    self.assertIn("cibuild", vars, "cibuild variable not detected")
    self.assertEqual(len(vars), 2, "False number of detected variables")
