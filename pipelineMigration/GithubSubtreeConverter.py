import re

from pipelineMigration import Job
from pipelineMigration.GithubConverter import GithubActionConverter


class GithubSubTreeConverter(GithubActionConverter):
    def __init__(self, pipeline, repoPath, repoID, compatibleImages: set = None):
        """
        :param pipeline: Pipeline object
        :param repoNames: IDs mapped to names of the repository
        :param repoPath: IDs mapped to paths to the repository
        """
        super().__init__(pipeline, compatibleImages)
        self.repoPath = repoPath
        self.repoID = repoID

    def parse_pipeline(self, name: str, secrets: list[str]) -> str:
        self.fileChangeJobNeeded = False
        pipelineString = ""
        pipelineString += f"name: {name}\n"
        pipelineString += "on:\n"
        pipelineString += "\tpush:\n"
        pipelineString += "\t\tpaths:\n"
        pipelineString += "\t\t\t- '" + self.repoPath + "/**'\n"
        pipelineString += "\tworkflow_dispatch:\n"
        pipelineString += "env:\n"
        pipelineString += f"\tCI_PROJECT_ID : {self.repoID}\n"
        if secrets:
            for secret in secrets:
                if type(secret) == tuple:
                    if secret[0] != "CI_PROJECT_ID":
                        pipelineString += f"\t{secret[0]} : " + f"{secret[1]}\n"
                else:
                    if secret != "CI_PROJECT_ID":
                        pipelineString += f"\t{secret} : " + "${{ secrets." + f"{secret}" + " }}\n"

        pipelineString += "jobs:\n"
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                self.fileChangeJobNeeded = True
                pipelineString += self.createFileChangeJob()
                break
        pipelineString += self.createStageJobs()
        for job in self.pipeline.jobs:
            pipelineString += self.parse_job(self.pipeline.jobs[job], secrets)
            pipelineString += "\n"
        return self.set_indentation_to_two_spaces(pipelineString)

    def parse_job(self, job: Job, secrets: list[str] = []) -> str:
        """
        Parses a job and returns it in the converted form as a string.
        :param job: Job object
        :param secrets: List of secrets to be used in the job
        :return: Converted job
        """
        jobString = super().parse_job(job, secrets)
        jobString = jobString.replace("            cd /workspace\n",
                                      "            cd /workspace\n" + "            cd " + f"{self.repoPath}" + "\n")
        patternRepo = r"(- name: Script\s+run: \|)"
        repoCD = r"\1\n" + f"          cd {self.repoPath}"
        jobString = re.sub(patternRepo, repoCD, jobString)
        # jobString = jobString.replace('${{ secrets.CI_PROJECT_ID }}', self.repoID)
        artifactUploadPattern = r"(- name: .*\n\s+uses: actions/upload-artifact@v4\n(?:\s+if: .*\n)?\s+with:\n(?:\s+.+\n)*\s+path: \|\n((?:\s+.+\n?)+))"

        prefix = f"{self.repoPath}/"

        def replace_paths_with_prefix(match):
            full_block = match.group(1)
            paths = match.group(2).splitlines()
            prefixed_paths = [f"            {prefix}{path.strip()}" for path in paths if
                              path.strip()]
            full_block_without_path = re.sub(r"path: \|.*", "", full_block, flags=re.DOTALL)
            return full_block_without_path + "path: |\n" + "\n".join(prefixed_paths) + "\n"

        jobString = re.sub(artifactUploadPattern, replace_paths_with_prefix, jobString)

        artifactDownloadPattern = r"(- name: .*\n\s+uses: actions/download-artifact@v4\n(?:\s+.+\n)*?\s+path: \|\n)((?:\s+.+\n?)+?)"

        jobString = re.sub(artifactDownloadPattern, replace_paths_with_prefix, jobString)

        uploadPagesPattern = r"(- name: Upload Pages\s+uses: actions/upload-pages-artifact@v3\s+with:\s+path: )(.+)"

        def add_prefix_to_upload_pages_path(match):
            full_block = match.group(1)  # Der Block bis einschließlich `path:`
            path_value = match.group(2).strip()  # Der ursprüngliche `path`-Wert
            return full_block + prefix + path_value  # Präfix hinzufügen

        jobString = re.sub(uploadPagesPattern, add_prefix_to_upload_pages_path, jobString)

        return jobString
