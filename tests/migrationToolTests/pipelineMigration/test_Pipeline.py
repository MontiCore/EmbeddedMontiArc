from unittest import TestCase

from migrationTool.pipelineMigration import Pipeline, Job


class TestPipeline(TestCase):
  def setUp(self):
    self.job_a = Job.Job("a", "ubuntu:latest", "First", [], [], "", [])
    self.job_b = Job.Job("b", "ubuntu:latest", "Second", [], ["a"], "", [])
    self.job_c = Job.Job("c", "ubuntu:latest", "Second", [], ["a"], "", [])
    self.pipeline = Pipeline.Pipeline(["First", "Second"], {"a": self.job_a, "b": self.job_b, "c": self.job_c},
                                      {"First": ["a"], "Second": ["b", "c"]}, ["First", "Second"])

  def test_get_previous_stage_jobs(self):
    previous_jobs = self.pipeline.get_previous_stage_jobs(self.job_b)
    self.assertEqual(previous_jobs, ["a"])
    previous_jobs = self.pipeline.get_previous_stage_jobs(self.job_c)
    self.assertEqual(previous_jobs, ["a"])

  def test_delete_job(self):
    self.pipeline.delete_job("b")
    self.assertNotIn("b", self.pipeline.jobs)
    self.assertNotIn("b", self.pipeline.stageJobs["Second"])
    self.assertEqual(self.pipeline.get_previous_stage_jobs(self.job_c), ["a"])
    self.assertEqual(self.pipeline.get_previous_stage_jobs(self.job_a), [])

    # Ensure that deleting a job does not affect other jobs
    self.assertIn("c", self.pipeline.jobs)
    self.assertIn("a", self.pipeline.jobs)

    self.pipeline.delete_job("c")
    self.assertNotIn("c", self.pipeline.jobs)
    self.assertNotIn("Second", self.pipeline.stageJobs)
    self.assertNotIn("Secdond", self.pipeline.stages)
    self.assertIn("First", self.pipeline.stageJobs)
    self.assertIn("First", self.pipeline.stages)
