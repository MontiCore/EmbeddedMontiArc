from pipelineMigration.Converter import Converter
from pipelineMigration.Job import Job
from pipelineMigration.Pipeline import Pipeline

class GithubActionConverter(Converter):
    """
    This class converts a pipeline Object to GitHub Actions.
    """


    def __init__(self, pipeline : Pipeline):
        self.pipeline = pipeline

        #ToDo: Read from config file
        self.compatibleImages = {"maven:3.6-jdk-8",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170-onnx:v0.0.1",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/tensorflow-onnx:latest",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/applications/mnistcalculator/tensorflow",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/mxnet/190:v0.0.2",
                                 "registry.git.rwth-aachen.de/monticore/embeddedmontiarc/generators/emadl2cpp/dockerimages/mxnet170:v0.0.1"
                                 }
        self.compatibleImages = {"maven:3.6-jdk-8"}
        self.timeout = 60

    def parsePipeline(self, name : str, secrets : list[str]) -> str:
        pipelineString = ""
        pipelineString += f"name: {name}\n"
        pipelineString += "on:\n"
        pipelineString += "\tpush:\n"
        pipelineString += "\tworkflow_dispatch:\n"
        pipelineString += "concurrency:\n"
        pipelineString += '\tgroup: "${{ github.ref }}"\n'
        pipelineString += '\tcancel-in-progress: true\n'
        if secrets:
            pipelineString += "env:\n"
            for secret in secrets:
                pipelineString += f"\t{secret} : " + "${{ secrets."+ f"{secret}"+" }}\n"

        pipelineString += "jobs:\n"

        for job in self.pipeline.jobs:
            pipelineString += self.parseJob(self.pipeline.jobs[job],secrets)
            pipelineString += "\n"
        return self.set_indentation_to_two_spaces(pipelineString)

    def parseJob(self, job : Job, secrets : list[str] = []) -> str:
        if job.image in self.compatibleImages:
            native = True
        else:
            native = False


        if job.needs:
            previousJobs = job.needs
        else:
            previousJobs = self.pipeline.getPreviousStageJobs(job)

        jobString=""
        jobString += f"\t{job.name}:\n"
        if previousJobs:
            jobString += (f"\t\tneeds: ")
            if len(previousJobs) == 1:
                jobString += f"{previousJobs[0]}\n"
            else:
                for i,j in enumerate(previousJobs):
                    if i == 0:
                        jobString += (f"[ {j} ")
                    else:
                        jobString += (f", {j}")
                jobString += (f"]\n")

        jobString += f"\t\truns-on: ubuntu-latest\n"
        if job.exc:
            for i,branch in enumerate(job.exc):
                if i == 0:
                    jobString += f"\t\tif: github.ref != 'refs/heads/{branch}'\n"
                else:
                    jobString += f"\t\t  && github.ref != 'refs/heads/{branch}'\n"

        if native:
            jobString += f"\t\tcontainer:\n"
            jobString += f"\t\t\timage: {job.image}\n"
        jobString += f"\t\ttimeout-minutes: {self.timeout}\n"
        jobString += f"\t\tsteps:\n"
        jobString += GithubActionConverter.__addCheckoutStep("DavidBlm/MNISTPipeline")
        #jobString += self.__addCheckoutStepManual("DavidBlm/MNISTPipeline")
        #jobString += GithubActionConverter.__restoreLargeFilesStep()
        jobString += GithubActionConverter.__restoreH5()
        if job.needs:
            for need in job.needs:
                if self.pipeline.jobs[need].artifacts:
                    jobString += GithubActionConverter.__downloadArtifacts(self.pipeline.jobs[need].artifacts["paths"], need)
        if not native:
            jobString += GithubActionConverter.__startDockerContainer(job.image, secrets, "")
        if native:
            jobString += f"\t\t\t- name: Script\n"
            #jobString += f"\t\t\t\tshell: bash\n"
            jobString += f"\t\t\t\trun: |\n"
            #jobString += f"\t\t\t\t\tcd repo\n"    #For manual clone
            for command in job.script:
                jobString += f"\t\t\t\t\t{command}\n"
        else:
            jobString += f"\t\t\t- name: Script\n"
            jobString += f"\t\t\t\tenv:\n"
            jobString += f"\t\t\t\t\tSCRIPT: |\n"
            jobString += f"\t\t\t\t\t\tcd /workspace\n"

            for command in job.script:
                jobString += f"\t\t\t\t\t\t{command}\n"
            jobString += f'\t\t\t\trun: docker exec build-container bash -c "$SCRIPT"\n'
        if job.artifacts:
            jobString += GithubActionConverter.__uploadArtifacs(job.artifacts["paths"],job.name)
        return Converter.set_indentation_to_two_spaces(jobString)

    @staticmethod
    def __addCheckoutStep(repo : str ="") -> str:
        checkout =""
        checkout += f"\t\t\t- name: Checkout latest commit\n"
        checkout += f"\t\t\t\tuses: actions/checkout@v4\n"
        checkout += f"\t\t\t\twith:\n"
        checkout += f"\t\t\t\t\tfetch-depth: 1\n"
        if repo:
            checkout += f"\t\t\t\t\trepository: {repo}\n"
            checkout +=  "\t\t\t\t\ttoken: ${{ secrets.ACCESS_TOKEN }}\n"
        return checkout

    @staticmethod
    def __addCheckoutStepManual(repo : str ="${{ github.repository }}") -> str:
        checkout =""
        checkout += f"\t\t\t- name: Checkout latest commit\n"
        checkout += f"\t\t\t\trun: |\n"
        checkout +=  "\t\t\t\t\tgit clone --depth 1 https://${{github.REPOSITORY_OWNER}}:${{ secrets.ACCESS_TOKEN }}@github.com/"+f"{repo} repo\n"
        checkout += f"\t\t\t\t\tcd repo\n"
        checkout += f"\t\t\t\t\tls\n"
        return checkout

    @staticmethod
    def __restoreLargeFilesStep() -> str:
        restore =""
        restore += f"\t\t\t- name: Restore large files\n"
        restore += f"\t\t\t\trun: |\n"
        #restore +=  "\t\t\t\t\tcd repo\n"   #For manual clone
        restore +=  "\t\t\t\t\tls\n"
        restore += f"\t\t\t\t\tfind . -type f -name '*.part*' | while read part; do\n"
        restore += f"\t\t\t\t\techo \"Restoring $part\"\n"
        restore += f"\t\t\t\t\tbase=$(echo \"$part\" | sed 's/.part.*//')\n"
        restore += f"\t\t\t\t\tcat \"$part\" >> \"$base\"\n"
        restore += f"\t\t\t\t\trm \"$part\"\n"
        restore += f"\t\t\t\t\tdone\n"
        return restore

    @staticmethod
    def __restoreH5() -> str:
        restore =""
        restore += f"\t\t\t- name: Restore large files\n"
        restore += f"\t\t\t\trun: |\n"
        #restore +=  "\t\t\t\t\tcd repo\n"    #For manual clone
        restore +=  "\t\t\t\t\tls\n"
        restore += "\t\t\t\t\tfind . -type f -name 'train_*' | while read file; do\n"
        restore += '\t\t\t\t\tdir=$(dirname "$file")\n'
        restore += '\t\t\t\t\t(cd "$dir" && cat train_* > train.h5)\n'
        restore += '\t\t\t\t\techo "Created train.h5 in $dir"\n'
        restore += "\t\t\t\t\tdone\n"
        return restore

    @staticmethod
    def __startDockerContainer(image : str, secrets : list[str], options : str) -> str:
        start = ""
        start += f"\t\t\t- name: Start Docker Container\n"
        start += f"\t\t\t\trun: |\n"
        start += f"\t\t\t\t\tdocker pull {image}\n"
        start += f"\t\t\t\t\tdocker run --name build-container -d -v $(pwd):/workspace --network=host {options}"
        for secret in secrets:
            start += f" -e {secret}=$"+"{{ secrets."+f"{secret}"+" }}"
        start += f" {image} tail -f /dev/null\n"
        return start

    @staticmethod
    def __uploadArtifacs(paths : list[str], name, expiration : int = 7) -> str:
        upload = ""
        upload += f"\t\t\t- name: Upload artifacts\n"
        upload += f"\t\t\t\tuses: actions/upload-artifact@v4\n"
        upload += f"\t\t\t\tif: success()\n"
        upload += f"\t\t\t\twith:\n"
        upload +=  f'\t\t\t\t\tname: {name}\n' #Todo: Change to previous job name
        upload += f"\t\t\t\t\tretention-days: 7\n" #Todo: Change to expiration
        upload += f"\t\t\t\t\tpath: |\n"
        for path in paths:
            upload += f"\t\t\t\t\t\t{path}\n"
        return upload

    @staticmethod
    def __downloadArtifacts(paths: list[str], name) -> str:
        download = ""
        download += f"\t\t\t- name: Download artifacts\n"
        download += f"\t\t\t\tuses: actions/download-artifact@v4\n"
        download += f"\t\t\t\twith:\n"
        download += f"\t\t\t\t\tname: {name}\n"
        download += f"\t\t\t\t\tpath: |\n"
        for path in paths:
            download += f"\t\t\t\t\t\t{path}\n"
        return download