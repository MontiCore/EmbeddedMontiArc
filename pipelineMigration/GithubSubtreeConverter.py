import re

from pipelineMigration import Job
from pipelineMigration.GithubConverter import GithubActionConverter


class GithubSubTreeConverter(GithubActionConverter):
    def __init__(self, pipeline, repoPath, repoID ):
        """
        :param pipeline: Pipeline object
        :param repoNames: IDs mapped to names of the repository
        :param repoPath: IDs mapped to paths to the repository
        """
        super().__init__(pipeline)
        self.repoPath = repoPath
        self.repoID = repoID


    def parsePipeline(self, name : str, secrets : list[str]) -> str:
        self.fileChangeJobNeeded = False
        pipelineString = ""
        pipelineString += f"name: {name}\n"
        pipelineString += "on:\n"
        pipelineString += "\tpush:\n"
        pipelineString += "\t\tpaths:\n"
        pipelineString += "\t\t\t- '" + self.repoPath+ "/**'\n"
        pipelineString += "\tworkflow_dispatch:\n"
        pipelineString += "concurrency:\n"
        pipelineString += '\tgroup: "${{ github.ref }}"\n'
        pipelineString += '\tcancel-in-progress: true\n'
        if secrets:
            pipelineString += "env:\n"
            for secret in secrets:
                pipelineString += f"\t{secret} : " + "${{ secrets." + f"{secret}" + " }}\n"

        pipelineString += "jobs:\n"
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                self.fileChangeJobNeeded = True
                pipelineString += self.createFileChangeJob()
                break
        pipelineString += self.createStageJobs()
        for job in self.pipeline.jobs:
            pipelineString += self.parseJob(self.pipeline.jobs[job], secrets)
            pipelineString += "\n"
        return self.set_indentation_to_two_spaces(pipelineString)

    def parseJob(self, job: Job, secrets: list[str] = []) -> str:
        """
        Parses a job and returns it in the converted form as a string.
        :param job: Job object
        :param secrets: List of secrets to be used in the job
        :return: Converted job
        """
        jobString = super().parseJob(job, secrets)
        jobString = jobString.replace("            cd /workspace\n", "            cd /workspace\n"+"            cd "+ f"{self.repoPath}"+"\n")
        patternRepo = r"(- name: Script\s+run: \|)"
        repoCD = r"\1\n" + f"          cd {self.repoPath}"
        jobString = re.sub(patternRepo, repoCD, jobString)
        jobString = jobString.replace('${{ secrets.CI_PROJECT_ID }}', self.repoID)
        #ToDo: Artifact upload to different path
        return jobString