from typing import TextIO
import  yaml

from pipelineMigration.Importer import Importer
from pipelineMigration.Pipeline import Pipeline
from pipelineMigration.Job import Job


class GitlabCIImporter(Importer):
    """
    Class for importing GitLab CI pipelines.
    """

    def __readStages(self) -> list[str]:
        """
        Reads the stages from the YAML data.
        :rtype: list[str]
        :return: Stages in the pipeline
        """
        return self.yamlData['stages']

    def __readJobs(self) -> dict[str, Job]:
        """
        Reads the jobs from the YAML data.
        :rtype: dict[str, Job]
        :return: Jobs in the pipeline
        """
        jobs = {}
        for name,parameter in self.yamlData.items():
            if "script" in parameter or "trigger" in parameter:
                jobs[name] = Job(
                    name=name,
                    image=parameter.get("image", ""),
                    stage=parameter.get("stage", ""),
                    script=parameter.get("script", []),
                    needs=parameter.get("needs", []),
                    when=parameter.get("when", ""),
                    exc=parameter.get("except", []),
                    artifacts=parameter.get("artifacts", []),
                    only = parameter.get("only", [])
                )
        return jobs


    def __readDependencies(self) -> tuple[dict[str,set[str]], dict[str, set[str]]]:
        """
        Reads the dependencies between jobs in the pipeline.
        :rtype: tuple[dict[str,set[str]], dict[str, set[str]]]
        :return:
        - stageJobs: Dictionary mapping each stage to its jobs
        - jobNeeds: Dictionary mapping each job to its immediate dependencies
        """
        stageJobs = {s : set() for s in self.stages}
        for jobName,jobParameter in self.jobs.items():
            stageJobs[jobParameter.stage].add(jobName)

        jobNeeds = {}
        for stage in self.stages:
            for jobName in stageJobs[stage]:
                if self.jobs[jobName].needs != []:
                    for j in self.jobs[jobName].needs:
                        if jobName in jobNeeds:
                            jobNeeds[jobName].add(j)
                        else:
                            jobNeeds[jobName] = {j}
        return stageJobs, jobNeeds

    def __getSchedule(self) -> list[str]:
        """
        Creates a schedule for the stages in the pipeline based on their dependencies.
        :rtype: list[str]
        :return: List with stage names in the order of execution
        """
        stageSchedule = []
        while len(stageSchedule) != len(self.stages):
            for stage, tasks in self.stageDependencies.items():
                if stage not in stageSchedule:
                    needsFullfilled = True
                    for job in tasks:
                        if job in self.needs:
                            for need in self.needs[job]:
                                if self.jobs[need].stage not in stageSchedule and need not in tasks:
                                    needsFullfilled = False
                                    break
                        if not needsFullfilled:
                            break
                    else:
                        stageSchedule.append(stage)
                        break
        return stageSchedule

    def getPipeline(self,file : TextIO) -> Pipeline:
        """
        Imports a pipeline from a GitLab CI YAML file and returns it as a Pipeline object.
        :param file: Path to the YAML file
        :type file: TextIO
        :rtype : Pipeline
        :return: Object representing the imported pipeline
        """
        self.yamlData = yaml.safe_load(file)
        self.stages = self.__readStages()
        self.jobs = self.__readJobs()
        self.stageDependencies, self.needs = self.__readDependencies()
        return Pipeline(self.stages, self.jobs, self.stageDependencies, self.needs, self.__getSchedule())