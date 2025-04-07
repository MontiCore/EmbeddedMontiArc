from abc import ABC, abstractmethod
from pipelineMigration.Pipeline import Pipeline

class Converter(ABC):
    """
    Abstract base class for Pipeline importers.
    """
    @abstractmethod
    def __init__(self, pipeline: Pipeline):
        self.pipeline = pipeline

    @abstractmethod
    def parsePipeline(self, **kwargs) -> str:
        """
        Parses the pipeline and returns it in the converted form as a string.
        :rtype: str
        :return: Converted pipeline
        """

    @abstractmethod
    def parseJob(self, job, **kwargs) -> str:
        """
        Parses the job and returns it in the converted form as a string.
        :rtype: str
        :return: Converted pipeline
        """

    @staticmethod
    def set_indentation_to_two_spaces(inputString : str) -> str:
        return inputString.replace('\t', '  ')