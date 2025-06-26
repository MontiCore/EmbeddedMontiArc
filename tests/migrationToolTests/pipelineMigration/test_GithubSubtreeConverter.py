import os
import shutil
from unittest import TestCase

from migrationTool.migration_types import Architecture
from migrationTool.pipelineMigration.GithubSubtreeConverter import GithubSubTreeConverter
from migrationTool.pipelineMigration.GitlabCIImporter import GitlabCIImporter

"""
DO NOT REFORMAT THIS FILE!
As the SubtreeConverter uses the normal GitHub converter, the jobs themselves are not tested here. Only subtree 
specific changes are tested.  
"""


class TestGithubSubTreeConverter(TestCase):
  def setUp(self):
    current_file_dir = os.path.dirname(__file__)
    testresources_path = os.path.abspath(os.path.join(current_file_dir, "../../testRessources/GitHubConverterSubtree"))
    shutil.copytree(testresources_path, os.path.join(os.getcwd(), "TEST"))
    self.architecture = Architecture.load_architecture(os.path.join(os.getcwd(), "TEST", "architecture.yaml"))
    with open(os.path.join(os.getcwd(), "TEST", ".gitlab-ci.yml")) as file:
      importer = GitlabCIImporter()
      self.pipeline = importer.getPipeline(file)
    compatibleImages = {"ubuntu:latest", "docker:latest"}
    self.github_converter = GithubSubTreeConverter(self.architecture, self.pipeline, "repoA", "1", compatibleImages)

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd(), "TEST"))

  def test_parse_pipeline(self):
    # Test that the pipeline is parsed correctly
    pipeline = self.github_converter.parse_pipeline("RepoA", self.architecture.get_repo_by_ID("1").secrets)
    self.assertIsNotNone(pipeline)
    self.assertIsInstance(pipeline, str)

    # Test that it contains the correct name

    self.assertIn("name: RepoA\n", pipeline)
    # Test that it declares triggers
    self.assertIn("on:\n", pipeline)
    push_path = """push:
    paths:
      - 'repoA/**'"""
    self.assertIn(push_path, pipeline, "Only a push to the subtree should trigger the pipeline")
    # Test that it contains the correct variables
    self.assertIn("env:\n  CI_PROJECT_ID : 1\n  CI_API_V4_URL : https://git.rwth-aachen.de/api/v4\n  GITLABTOKEN : ${{ "
                  "secrets.GITLABTOKEN }}\n  TEST "
                  ": test", pipeline)
    # Test that it contains the correct jobs
    self.assertIn("FileChanges", pipeline)
    self.assertIn("test_phase", pipeline)
    self.assertIn("build_phase", pipeline)
    self.assertNotIn("deploy_phase", pipeline)
    self.assertIn("variable_job", pipeline)
    self.assertIn("artifact_job", pipeline)
    self.assertIn("artifact_advanced_job", pipeline)
    self.assertIn("image_native_job", pipeline)
    self.assertIn("image_manual_migrated_job", pipeline)
    self.assertIn("image_manual_not_migrated_job", pipeline)
    self.assertIn("only_branch_job", pipeline)
    self.assertIn("except_branch_job", pipeline)
    self.assertIn("only_files_job", pipeline)
    self.assertIn("except_files_job", pipeline)
    self.assertIn("only_except_files_job", pipeline)
    self.assertIn("need_job", pipeline)
    self.assertIn("pages", pipeline)
    self.assertIn("trigger_job", pipeline)
    self.assertIn("docker_not_migrated_job", pipeline)
    self.assertIn("docker_migrated_job", pipeline)
    self.assertIn("maven_job", pipeline)
    self.assertIn("report_job", pipeline)
    self.assertIn("rules_job", pipeline)
    self.assertIn("allow_failure_job", pipeline)
    self.assertIn("dependencies_job", pipeline)
    # Test that FileChange job provides correct outputs
    output_definition = """outputs:
      runonly_files_job: ${{steps.only_files_job.outputs.run}}
      runonly_except_files_job: ${{steps.only_except_files_job.outputs.run}}
      runrules_job: ${{steps.rules_job.outputs.run}}"""
    self.assertIn(output_definition, pipeline)

  def test_parse_variable_job(self):
    # Test that the variable job is parsed correctly
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["variable_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    cd_to_reop = """run: |
            cd repoA"""
    self.assertIn(cd_to_reop, pipeline)

  def test_parse_artifact_job(self):
    # Test that the artifact job is parsed correctly
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["artifact_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    print(pipeline)
    updated_path = """path: |
            repoA/artifact.txt"""
    self.assertIn(updated_path, pipeline)
    cd_to_reop = """run: |
            cd repoA"""
    self.assertIn(cd_to_reop, pipeline)

  def test_parse_need_job(self):
    # Test that the need job is parsed correctly
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["need_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    updated_path = """path: |
            repoA/artifact.txt"""
    self.assertIn(updated_path, pipeline)
    cd_to_reop = """run: |
            cd repoA"""
    self.assertIn(cd_to_reop, pipeline)

  def test_image_manual_migrated_job(self):
    # Test that the native image job is parsed correctly
    pipeline = self.github_converter.parse_job(self.pipeline.jobs["image_manual_migrated_job"],
                                               self.architecture.get_repo_by_ID("1").secrets)
    cd_to_repo = """SCRIPT: |
            cd /workspace
            cd repoA"""
    self.assertIn(cd_to_repo, pipeline)

  def test_parse_trigger_job(self):
    # Test that the trigger job is parsed correctly
    self.skipTest("To be implemented")
