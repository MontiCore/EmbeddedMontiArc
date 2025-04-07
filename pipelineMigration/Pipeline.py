class Pipeline:
    """
    Class representing a CI/CD pipeline.
    """

    def __init__(self, stages, jobs, stageJobs, jobNeeds,schedule):
        """
        Initializes the pipeline with the given stages, jobs, and their dependencies.
        :param stages: List of stages in the pipeline
        :param jobs: Dictionary of jobs in the pipeline
        :param stageJobs: Dictionary mapping each stage to its jobs
        :param jobNeeds: Dictionary mapping each job to its dependencies
        :param schedule: List representing the order of stages in the pipeline
        """

        self.stages = stages
        self.jobs = jobs
        self.stageJobs = stageJobs
        self.jobNeeds = jobNeeds
        self.schedule = schedule


    def __str__(self):
        string = f"Stages: {self.stages}\n"

        for job in self.jobs.values():
            string += "-------------------\n"
            string += str(job)
        return string

    def getPreviousStageJobs(self,job):
        """
        Returns the names of the jobs that have to be finished before the given job can start.
        :param job: Name of job
        :return: Previous jobs
        """
        jobsFinished = []
        sequenceNumber = self.schedule.index(job.stage)
        for i in range(sequenceNumber):
            for j in self.stageJobs[self.schedule[i]]:
                jobsFinished.append(j)
        return jobsFinished