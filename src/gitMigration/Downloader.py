from abc import ABC

from src.Architecture import Architecture
from src.Config import Config


class Downloader(ABC):
    def __init__(self, config : Config):
        self.config = config

    def clone(self):
        """
        Clone all repositories with all branches from the source instance.
        """
        pass

    def scan(self):
        """
        Scan all repositories with all branches from the source instance.
        """
        pass