import os
import shutil
from unittest import TestCase

from migrationTool.pipelineMigration import GitlabCIImporter
from migrationTool.pipelineMigration.Job import Job
from migrationTool.pipelineMigration.Pipeline import Pipeline


class TestGitlabCIImporter(TestCase):
  def setUp(self):
    current_file_dir = os.path.dirname(__file__)
    testresources_path = os.path.abspath(os.path.join(current_file_dir, "../../testRessources/GitlabImporter"))
    shutil.copytree(testresources_path, os.path.join(os.getcwd(), "TEST"))

  def tearDown(self):
    shutil.rmtree(os.path.join(os.getcwd(), "TEST"))

  def test_get_pipeline(self):
    importer = GitlabCIImporter.GitlabCIImporter()
    with open(os.path.join(os.getcwd(), "TEST", ".gitlab-ci.yml")) as file:
      pipeline = importer.getPipeline(file)

    self.assertIsNotNone(pipeline)
    self.assertIsInstance(pipeline, Pipeline)
    self.assertEqual({"build", "test", "deploy", "pages"}, set(pipeline.stages))  # Correct stages
    self.assertEqual(["test", "build", "deploy", "pages"], pipeline.schedule)  # Correct order of stages
    self.assertEqual({"TEST": "test"}, pipeline.variables)  # Correct variables
    correct_stageJobs = {'build': {'build_job'}, 'deploy': {'deploy_job'}, 'pages': {'pages'},
                         'test': {'test2_job', 'test_job'}}
    self.assertEqual(correct_stageJobs, pipeline.stageJobs)
    # Check jobs
    self.assertEqual(len(pipeline.jobs), 5)
    correct_jobs = {
      "test_job": "Name: test_job\nImage: docker:latest\nStage: test\nScript: ['echo \"This is a before script\"', "
                  "'echo \"This is a before script for test_job\"', 'echo \"This is a test job\"']\n",
      "test2_job": "Name: test2_job\nImage: docker:latest\nStage: test\nScript: ['echo \"This is a before script\"', "
                   "'echo \"This is another test job\"']\nWhen: always\nAllow Failure: True\n",
      "build_job": "Name: build_job\nImage: docker:latest\nStage: build\nScript: ['echo \"This is a before script\"', "
                   "'echo \"This is a build job\"']\nNeeds: ['test2_job', 'test_job']\nExcept: ['docu']\n",
      "deploy_job": "Name: deploy_job\nImage: docker:latest\nStage: deploy\nScript: ['echo \"This is a before "
                    "script\"']\nArtifacts: {'paths': ['abc/**']}\nOnly: ['master']\nTrigger: {'include': ["
                    "{'project': 'architecture', 'file': '/.gitlab-ci.yml'}]}\n",
      "pages": "Name: pages\nImage: docker:latest\nStage: pages\nScript: ['echo \"This is a before script\"', "
               "'echo \"This is a pages job\"']\nArtifacts: {'paths': ['public/']}\nRules: ['$CI_COMMIT_BRANCH == "
               "\"main\"']\n"}
    for job in pipeline.jobs:
      self.assertEqual(correct_jobs[job], str(pipeline.jobs[job]))
