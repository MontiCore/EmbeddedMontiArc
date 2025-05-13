from pipelineMigration.Converter import Converter
from pipelineMigration.Job import Job
from pipelineMigration.Pipeline import Pipeline


class GithubActionConverter(Converter):
    """
    This class converts a pipeline Object to GitHub Actions.
    """

    def __init__(self, pipeline: Pipeline, compatibleImages: set = None):
        self.pipeline = pipeline

        self.compatibleImages = {"maven:3.6-jdk-8",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170-onnx:v0.0.1",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/tensorflow-onnx:latest",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/mnistcalculator/tensorflow",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/mxnet/190:v0.0.2",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170:v0.0.1"
                                 }
        self.compatibleImages = {"maven:3.6-jdk-8"}
        self.timeout = 60

    def parse_pipeline(self, name: str, secrets: list[str]) -> str:
        self.fileChangeJobNeeded = False
        pipelineString = ""
        pipelineString += f"name: {name}\n"
        pipelineString += "on:\n"
        pipelineString += "\tpush:\n"
        pipelineString += "\tworkflow_dispatch:\n"
        pipelineString += "env:\n"
        if secrets:
            for secret in secrets:
                if type(secret) == tuple:
                    pipelineString += f"\t{secret[0]} : " + f"{secret[1]}\n"
                else:
                    pipelineString += f"\t{secret} : " + "${{ secrets." + f"{secret}" + " }}\n"

        pipelineString += "jobs:\n"
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                pipelineString += self.createFileChangeJob()
                self.fileChangeJobNeeded = True
                break
        pipelineString += self.createStageJobs()

        for job in self.pipeline.jobs:
            pipelineString += self.parse_job(self.pipeline.jobs[job], secrets)
            pipelineString += "\n"
        return self.set_indentation_to_two_spaces(pipelineString)

    def parse_job(self, job: Job, secrets: list[str] = []) -> str:
        if job.image in self.compatibleImages or not job.image:
            native = True
        else:
            native = False
        jobString = ""
        jobString += f"\t{job.name.replace("/", "_").replace(" ", "_")}:\n"
        if job.needs:
            jobString += (f"\t\tneeds: ")
            if len(job.needs) == 1:
                jobString += f"{job.needs[0].replace("/", "_").replace(" ", "_")}\n"
            else:
                for i, j in enumerate(job.needs):
                    if i == 0:
                        jobString += (f"[ {j.replace("/", "_").replace(" ", "_")} ")
                    else:
                        jobString += (f", {j.replace("/", "_").replace(" ", "_")}")
                jobString += (f"]\n")
        else:
            i = self.pipeline.stages.index(job.stage)
            if i > 0:
                jobString += f"\t\tneeds: {self.pipeline.schedule[i - 1].replace('/', '_').replace(' ', '_') + "_phase"}\n"
            else:
                if self.fileChangeJobNeeded:
                    jobString += f"\t\tneeds: FileChanges\n"

        jobString += self.ifCondition(job)

        jobString += f"\t\truns-on: ubuntu-latest\n"

        if native and job.image:
            jobString += f"\t\tcontainer:\n"
            jobString += f"\t\t\timage: {job.image}\n"
        jobString += f"\t\ttimeout-minutes: {self.timeout}\n"
        if job.artifacts:
            if job.artifacts["paths"] == ["public"] and job.name == "pages":
                jobString += f"\t\tpermissions:\n"
                jobString += f"\t\t\tpages: write\n"
                jobString += f"\t\t\tid-token: write\n"
        jobString += f"\t\tsteps:\n"
        jobString += GithubActionConverter.addCheckoutStep()
        # jobString += GithubActionConverter.__addCheckoutStep("DavidBlm/MNISTPipeline")
        # jobString += self.__addCheckoutStepManual("DavidBlm/MNISTPipeline")
        jobString += GithubActionConverter.restoreLargeFilesStep()
        # jobString += GithubActionConverter.__restoreH5()
        if job.needs:
            for need in job.needs:
                if self.pipeline.jobs[need].artifacts:
                    jobString += GithubActionConverter.downloadArtifacts(self.pipeline.jobs[need].artifacts["paths"],
                                                                         need.replace("/", "_").replace(" ", "_"))
        if not native:
            jobString += GithubActionConverter.startDockerContainer(job.image, secrets, "")
        if native:
            jobString += f"\t\t\t- name: Script\n"
            if job.allowFailure == True:
                jobString += "\t\t\t\tcontinue-on-error: true\n"
            # jobString += f"\t\t\t\tshell: bash\n"
            jobString += f"\t\t\t\trun: |\n"
            # jobString += f"\t\t\t\t\tcd repo\n"    #For manual clone
            for command in job.script:
                command = self.scriptParser(command)
                jobString += f"\t\t\t\t\t{command}\n"
        else:
            jobString += f"\t\t\t- name: Script\n"
            if job.allowFailure == True:
                jobString += "\t\t\t\tcontinue-on-error: true\n"
            jobString += f"\t\t\t\tenv:\n"
            jobString += f"\t\t\t\t\tSCRIPT: |\n"
            jobString += f"\t\t\t\t\t\tcd /workspace\n"

            for command in job.script:
                command = self.scriptParser(command)
                jobString += f"\t\t\t\t\t\t{command}\n"
            jobString += f'\t\t\t\trun: docker exec build-container bash -c "$SCRIPT"\n'
        if job.artifacts:
            if job.artifacts["paths"] == ["public"] and job.name == "pages":
                jobString += GithubActionConverter.deployPages(job.artifacts["paths"][0])
            else:
                jobString += GithubActionConverter.uploadArtifacs(job.artifacts["paths"],
                                                                  job.name.replace("/", "_").replace(" ", "_"),
                                                                  job.artifacts["expire_in"])
        return Converter.set_indentation_to_two_spaces(jobString)

    @staticmethod
    def addCheckoutStep(repo: str = "", depth=1) -> str:
        checkout = ""
        checkout += f"\t\t\t- name: Checkout latest commit\n"
        checkout += f"\t\t\t\tuses: actions/checkout@v4\n"
        checkout += f"\t\t\t\twith:\n"
        checkout += f"\t\t\t\t\tfetch-depth: {depth}\n"
        if repo:
            checkout += f"\t\t\t\t\trepository: {repo}\n"
            checkout += "\t\t\t\t\ttoken: ${{ secrets.ACCESS_TOKEN }}\n"
        return checkout

    @staticmethod
    def addCheckoutStepManual(repo: str = "${{ github.repository }}") -> str:
        checkout = ""
        checkout += f"\t\t\t- name: Checkout latest commit\n"
        checkout += f"\t\t\t\trun: |\n"
        checkout += "\t\t\t\t\tgit clone --depth 1 " + "https://${{github.REPOSITORY_OWNER}}:${{ secrets.ACCESS_TOKEN }}@github.com/" + f"{repo} repo\n"
        checkout += f"\t\t\t\t\tcd repo\n"
        checkout += f"\t\t\t\t\tls\n"
        return checkout

    @staticmethod
    def restoreLargeFilesStep() -> str:
        restore = ""
        restore += f"\t\t\t- name: Restore large files\n"
        restore += f"\t\t\t\trun: |\n"
        # restore +=  "\t\t\t\t\tcd repo\n"   #For manual clone
        restore += "\t\t\t\t\tls\n"
        restore += f"\t\t\t\t\tfind . -type f -name '*.part*' | sort | while read part; do\n"
        restore += f"\t\t\t\t\techo \"Restoring $part\"\n"
        restore += f"\t\t\t\t\tbase=$(echo \"$part\" | sed 's/.part.*//')\n"
        restore += f"\t\t\t\t\tcat \"$part\" >> \"$base\"\n"
        restore += f"\t\t\t\t\trm \"$part\"\n"
        restore += f"\t\t\t\t\tdone\n"
        return restore

    @staticmethod
    def restoreH5() -> str:
        restore = ""
        restore += f"\t\t\t- name: Restore large files\n"
        restore += f"\t\t\t\trun: |\n"
        # restore +=  "\t\t\t\t\tcd repo\n"    #For manual clone
        restore += "\t\t\t\t\tls\n"
        restore += "\t\t\t\t\tfind . -type f -name 'train_*' | while read file; do\n"
        restore += '\t\t\t\t\tdir=$(dirname "$file")\n'
        restore += '\t\t\t\t\t(cd "$dir" && cat train_* > train.h5)\n'
        restore += '\t\t\t\t\techo "Created train.h5 in $dir"\n'
        restore += "\t\t\t\t\tdone\n"
        return restore

    @staticmethod
    def startDockerContainer(image: str, secrets: list[str], options: str) -> str:
        start = ""
        start += f"\t\t\t- name: Start Docker Container\n"
        start += f"\t\t\t\trun: |\n"
        if "ghcr.io" in image:
            start += '\t\t\t\t\techo "${{ secrets.GITHUB_TOKEN }}" | docker login ghcr.io -u "${{ github.actor }}" --password-stdin\n'
        start += f"\t\t\t\t\tdocker pull {image}\n"
        start += f"\t\t\t\t\tdocker run --name build-container -d -v $(pwd):/workspace --network=host {options}"
        for secret in secrets:
            if type(secret) == str:
                if secret != "CI_PROJECT_ID":
                    start += f" -e {secret}=$" + "{{ secrets." + f"{secret}" + " }}"
                else:
                    start += f" -e {secret}=${secret}"
            else:
                start += f" -e {secret[0]}=${secret[0]}"
        start += f" {image} tail -f /dev/null\n"
        return start

    @staticmethod
    def uploadArtifacs(paths: list[str], name, expiration: int = 7) -> str:
        upload = ""
        upload += f"\t\t\t- name: Upload artifacts\n"
        upload += f"\t\t\t\tuses: actions/upload-artifact@v4\n"
        upload += f"\t\t\t\tif: success()\n"
        upload += f"\t\t\t\twith:\n"
        upload += f'\t\t\t\t\tname: {name}\n'
        upload += f"\t\t\t\t\tretention-days: {expiration}\n"  # Todo: Change to expiration, needs to be tested
        upload += f"\t\t\t\t\tpath: |\n"
        for path in paths:
            upload += f"\t\t\t\t\t\t{path}\n"
        return upload

    @staticmethod
    def downloadArtifacts(paths: list[str], name) -> str:
        download = ""
        download += f"\t\t\t- name: Download artifacts\n"
        download += f"\t\t\t\tuses: actions/download-artifact@v4\n"
        download += f"\t\t\t\twith:\n"
        download += f"\t\t\t\t\tname: {name}\n"
        download += f"\t\t\t\t\tpath: |\n"
        for path in paths:
            download += f"\t\t\t\t\t\t{path}\n"
        return download

    @staticmethod
    def deployPages(path: str) -> str:
        deploy = ""
        deploy += "\t\t\t- name: Upload Pages\n"
        deploy += "\t\t\t\tuses: actions/upload-pages-artifact@v3\n"
        deploy += "\t\t\t\twith:\n"
        deploy += f"\t\t\t\t\tpath: {path}/\n"
        deploy += "\t\t\t- name: Deploy to Pages\n"
        deploy += "\t\t\t\tuses: actions/deploy-pages@v4\n"
        return deploy

    def ifCondition(self, job: Job) -> str:
        ifString = ""
        if self.pipeline.stages.index(job.stage) != 0:
            ifString += "\t\tif: ${{ !cancelled() && !contains(needs.*.result, 'failure') "

        # Handle only
        if job.only:
            if type(job.only) == dict:
                if "changes" in job.only:
                    if ifString == "":
                        ifString += "\t\tif: ${{"
                    else:
                        ifString += " && "
                    ifString += f"needs.FileChanges.outputs.run{job.name.replace('/', '_').replace(' ', '_')} == 'true'"
            if type(job.only) == list:
                if ifString == "":
                    ifString += "\t\tif: ${{"
                else:
                    ifString += " && "
                for i, branch in enumerate(job.only):
                    if i == 0:
                        ifString += f" github.ref == 'refs/heads/{branch}'"
                    else:
                        ifString += f" && github.ref == 'refs/heads/{branch}'"
        # Handle except
        if job.exc:
            if type(job.exc) == dict:
                if ifString == "":
                    ifString += "\t\tif: ${{"
                if "changes" in job.exc:
                    ifString += f" && !contains(github.event.head_commit.message, '"
                    for i, p in enumerate(job.exc['changes']):
                        if i == 0:
                            ifString += f"{p}"
                        else:
                            ifString += f", {p}"
                    ifString += "')"

            if type(job.exc) == list:
                for i, branch in enumerate(job.exc):
                    if i == 0 and not ifString:
                        ifString += f"\t\tif: github.ref != 'refs/heads/{branch}'"
                    else:
                        ifString += f"\t\t  && github.ref != 'refs/heads/{branch}'"
        if ifString:
            ifString += "}}\n"
        return ifString

    def createStageJobs(self):
        lastStage = ""
        jobString = ""
        for stage in self.pipeline.schedule[:-1]:
            jobString += f"\t{stage + "_phase:"}\n"

            jobString += f"\t\tneeds: ["
            if lastStage:
                jobString += f'{lastStage + "_phase, "}'
            lastStage = stage
            for i, job in enumerate(self.pipeline.stageJobs[stage]):
                if i == 0:
                    jobString += f"{job.replace('/', '_').replace(' ', '_')}"
                else:
                    jobString += f", {job.replace('/', '_').replace(' ', '_')}"
            jobString += f"]\n"
            jobString += "\t\tif: ${{ !cancelled()"
            # jobString += " && !contains(needs.*.result, 'failure')}}\n"
            jobString += "}}\n"
            jobString += f"\t\truns-on: ubuntu-latest\n"
            jobString += "\t\tsteps:\n"
            jobString += f"\t\t\t\t- run: |\n"
            jobString += f'\t\t\t\t\t\techo "Finished stage {stage}"\n'
            jobString += "\t\t\t\t  if: ${{!contains(needs.*.result, 'failure')}}\n"
            jobString += f"\t\t\t\t- run: |\n"
            jobString += f'\t\t\t\t\t\techo "Failed stage {stage}"\n'
            jobString += "\t\t\t\t\t\texit 1\n"
            jobString += "\t\t\t\t  if: ${{contains(needs.*.result, 'failure')}}\n"
        jobString += "\n"
        return jobString

    def createFileChangeJob(self):
        jobString = "\tFileChanges:\n"
        jobString += f"\t\truns-on: ubuntu-latest\n"
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                jobString += f"\t\toutputs:\n"
                break
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                jobString += f"\t\t\trun{job.name}: " + "${{" + f"steps.{job.name.replace('/', '_').replace(' ', '_')}.outputs.run" + "}}\n"
        jobString += "\t\tsteps:\n"
        jobString += self.addCheckoutStep(depth=2)
        jobString += f"\t\t\t- name: Check for file changes\n"
        jobString += f"\t\t\t\trun: |\n"
        jobString += f"\t\t\t\t\tCHANGES=$(git diff --name-only HEAD^ HEAD)\n"
        jobString += '\t\t\t\t\techo "$CHANGES"\n'
        jobString += '\t\t\t\t\techo "$CHANGES" > diff.txt\n'
        for _, job in self.pipeline.jobs.items():
            if job.only and type(job.only) == dict and "changes" in job.only:
                jobString += f"\t\t\t- name: Check {job.name}\n"
                jobString += f"\t\t\t\tid: {job.name.replace('/', '_').replace(' ', '_')}\n"
                jobString += f"\t\t\t\trun: |\n"
                jobString += "\t\t\t\t\trun=false\n"
                for path in job.only["changes"]:
                    jobString += "\t\t\t\t\tif cat diff.txt | grep" + f" '^{path.replace("/**/*", "")}'" + "; then\n"
                    jobString += '\t\t\t\t\t\techo "RUN"\n'
                    jobString += "\t\t\t\t\t\trun=true\n"
                    jobString += "\t\t\t\t\telse\n"
                    jobString += '\t\t\t\t\t\techo "DONT RUN"\n'
                    jobString += "\t\t\t\t\tfi\n"
                jobString += '\t\t\t\t\techo "run=$run" >> $GITHUB_OUTPUT\n'
        return jobString

    def scriptParser(self, script: str) -> str:
        """
        Parses the script of a job and returns it as a string.
        :param script: Script to be parsed
        :return: Parsed script
        """
        if "mvn" in script:
            script += " -Dmaven.wagon.http.retryHandler.count=50 -Dmaven.wagon.http.connectionTimeout=6000000 -Dmaven.wagon.http.readTimeout=600000000"
        # ToDo: Delete for production
        if "deploy" in script:
            return
        script = script.replace("${CI_JOB_TOKEN}", "${{ secrets.GITLABTOKEN }}")
        script = script.replace("$DOCKER_TOKEN", "${{ secrets.GITLABTOKEN }}")
        script = script.replace("$CI_REGISTRY_PASSWORD", "${{ secrets.GITLABTOKEN }}")
        # ToDo: Add docker, user replacement

        return script
