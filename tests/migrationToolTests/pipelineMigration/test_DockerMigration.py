import os
import shutil
from unittest import TestCase
from rich.progress import Progress
from unittest.mock import MagicMock, patch

from migrationTool.migration_types import Architecture, Config
from migrationTool.pipelineMigration import DockerMigration


class TestDockerMigration(TestCase):
  @patch("gitlab.Gitlab")
  def setUp(self, mock_gitlab):
    mock_gitlab.return_value.auth = MagicMock(return_value=None)
    path = os.getcwd()
    path = path.split(os.path.sep)
    for i in range(len(path)):
      if path[i] == "tests":
        path = os.path.sep.join(path[:i + 1])
        break
    shutil.copytree(os.path.join(path, "testRessources", "DockerMigration"), os.path.join(os.getcwd(), "TEST"))
    docker_migration_path = os.path.join(os.getcwd(), "TEST")
    architecture = Architecture.load_architecture(os.path.join(docker_migration_path, "architecture.yaml"))
    config = Config(os.path.join(docker_migration_path, "config.yaml"))
    self.docker_migration = DockerMigration.DockerMigration(architecture, config,
                                                            os.path.join(docker_migration_path, "images.txt"))

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd(), "TEST"))

  def test_add_images_being_migrated(self):
    # Test that method correctly identifies images and constructs URLs
    images = self.docker_migration.add_images_being_migrated()

    # Evaluate
    correct = {
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc:latest':
        'ghcr.io/davidblm/RepoA/embedded_montiarc:latest',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc:0.1.0':
        'ghcr.io/davidblm/RepoA/embedded_montiarc:0.1.0',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/test:latest':
        'ghcr.io/davidblm/RepoA/test:latest',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/gans/repob:latest':
        'ghcr.io/davidblm/RepoB:latest'}
    self.assertIsInstance(images, dict)
    self.assertDictEqual(correct, images)

  @patch('rich.prompt.Confirm.ask')  # No Keyboard during test
  def test_write_images_being_migrated(self, mock_confirm):
    # Test that method writes the correct images to the file
    mock_confirm.side_effect = [True, False, True, False]
    self.docker_migration.write_images_being_migrated()

    # Evaluate
    with open(self.docker_migration.path, "r") as file:
      content = file.read().splitlines()
    self.assertEqual(len(content), 8)
    # Existing content should be preserved, so 4 existing lines + 4 new lines
    self.assertEqual(content[0], 'ubuntu:22.04;ubuntu:22.04;y')
    self.assertEqual(content[1],
                     'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2pythonwrapper/tests/mvn'
                     '-swig:latest;ghcr.io/davidblm/EMADL2PythonWrapper/tests/mvn-swig:latest;n')
    self.assertEqual(content[2],
                     'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2someip:latest;ghcr.io'
                     '/davidblm/EMAM2SomeIP:latest;n')
    self.assertEqual(content[3],
                     'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2someip:v2;ghcr.io'
                     '/davidblm/EMAM2SomeIP:v2;n')
    new_lines = set(content[4:])
    correct_new_lines = {
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc:latest;ghcr.io'
      '/davidblm/RepoA/embedded_montiarc:latest;y',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc:0.1.0;ghcr.io'
      '/davidblm/RepoA/embedded_montiarc:0.1.0;n',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/test:latest;ghcr.io/davidblm/RepoA'
      '/test:latest;y',
      'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/gans/repob:latest;ghcr.io/davidblm/RepoB'
      ':latest;n'}
    self.assertEqual(new_lines, correct_new_lines)

  def test_read_previously_migrated_docker_images(self):
    # Test that method reads previously migrated images correctly
    images = self.docker_migration.read_previously_migrated_docker_images()

    # Evaluate
    correct = {'ubuntu:22.04': 'ubuntu:22.04',
               'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2pythonwrapper/tests/mvn-swig'
               ':latest': 'ghcr.io/davidblm/EMADL2PythonWrapper/tests/mvn-swig:latest',
               'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2someip:latest':
                 'ghcr.io/davidblm/EMAM2SomeIP:latest',
               'registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emam2someip:v2':
                 'ghcr.io/davidblm/EMAM2SomeIP:v2'}
    self.assertIsInstance(images, dict)
    self.assertDictEqual(correct, images)
    self.assertEqual(self.docker_migration.nativeImage, {"ubuntu:22.04"})

  @patch('rich.prompt.Confirm.ask')
  def test_get_new_image(self, mock_confirm):
    # Test that method returns correct new image URL or original if not found
    mock_confirm.side_effect = [True, False, False, False, False, True]

    with Progress() as progress:
      task = progress.add_task("[cyan]Testing...", total=100)
      result = self.docker_migration.get_new_Image(progress, "ubuntu:22.04")
      self.assertEqual(result, ('', "ubuntu:22.04"))

      result = self.docker_migration.get_new_Image(progress,
                                                   "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/repoa/embedded_montiarc:latest")
      self.assertEqual(result, ('', 'ghcr.io/davidblm/RepoA/embedded_montiarc:latest'))
      self.assertIn("ghcr.io/davidblm/RepoA/embedded_montiarc:latest", self.docker_migration.nativeImage)

      result = self.docker_migration.get_new_Image(progress,
                                                   "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/gans/repob:latest")
      self.assertEqual(result, ('', 'ghcr.io/davidblm/RepoB:latest'))
      self.assertIn("ghcr.io/davidblm/RepoA/embedded_montiarc:latest", self.docker_migration.nativeImage)

      result = self.docker_migration.get_new_Image(progress,
                                                   "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2pythonwrapper/tests/mvn-swig:latest")
      self.assertEqual(result, ('', 'ghcr.io/davidblm/EMADL2PythonWrapper/tests/mvn-swig:latest'))

      result = self.docker_migration.get_new_Image(progress, "registry.git.rwth-aachen.de/tests/not_migrated:latest")
      self.assertEqual(result, ('', "registry.git.rwth-aachen.de/tests/not_migrated:latest"))
      result = self.docker_migration.get_new_Image(progress, "maven:latest")
      self.assertEqual(result, ('', "maven:latest"))
      self.assertIn("maven:latest", self.docker_migration.notNativeImage)
      result = self.docker_migration.get_new_Image(progress, "maven2:latest")
      self.assertEqual(result, ('', "maven2:latest"))
      self.assertIn("maven2:latest", self.docker_migration.nativeImage)
