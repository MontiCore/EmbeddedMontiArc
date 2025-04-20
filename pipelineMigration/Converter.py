import os
from abc import ABC, abstractmethod
from pipelineMigration.Pipeline import Pipeline
import shutil
import re

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

    @staticmethod
    def replace_job_token_in_settings(file_path):
        """
        Ersetzt 'Job-Token' durch 'Private-Token' in der settings.xml-Datei.
        :param file_path: Pfad zur settings.xml-Datei
        """
        with open(file_path, 'r', errors='ignore') as file:
            content = file.read()

        # Überprüfen, ob 'Job-Token' vorhanden ist
        if 'Job-Token' or 'Private-Token' in content:
            # Backup der Originaldatei erstellen
            backup_path = file_path + '.bak'
            shutil.copy(file_path, backup_path)
            #print(f"Backup erstellt: {backup_path}")

            # 'Job-Token' durch 'Private-Token' ersetzen
            updated_content = content.replace('Job-Token', 'Private-Token')
            #updated_content = updated_content.replace("env.CI_JOB_TOKEN", "env.GITLABTOKEN")
            #updated_content = updated_content.replace("CI_JOB_TOKEN", "env.GITLABTOKEN")
            pattern = r"""
            <property>\s*
            <name>Private-Token</name>\s*
            <value>.*?</value>\s*
            </property>
            """

            def replace_value(match):
                return re.sub(r"<value>.*?</value>", "<value>${env.GITLABTOKEN}</value>", match.group(0))

            updated_content = re.sub(pattern, replace_value, updated_content, flags=re.DOTALL | re.VERBOSE)


            # Datei mit den Änderungen speichern
            with open(file_path, 'w') as file:
                file.write(updated_content)
            print(f"'Job-Token' ersetzt in Datei: {file_path}")
        else:
            print(f"Kein 'Job-Token' gefunden in: {file_path}")

    @staticmethod
    def process_settings_files(repo_path):
        """
        Durchsucht das Repository nach settings.xml-Dateien und ersetzt 'Job-Token'.
        :param repo_path: Pfad zum Repository
        """
        for root, _, files in os.walk(repo_path):
            for file in files:
                if file == 'settings.xml' or file == "ci_settings.xml":
                    file_path = os.path.join(root, file)
                    Converter.replace_job_token_in_settings(file_path)