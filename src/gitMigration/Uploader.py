import logging
import os
from abc import ABC
from datetime import time, datetime

import yaml
import git

from src.Architecture import Architecture
from src.Config import Config

logger = logging.getLogger(__name__)


class Uploader(ABC):
    def __init__(self, config : Config, architecture : Architecture):
        self.architecture = architecture
        self.config = config

        self.namespaces = {}
        self.repoNames = {}
        self.branchesToBeMigrated = {}
        for repoID in data.keys():
            if data[repoID]["Branches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["StaleBranches"]
            elif data[repoID]["StaleBranches"] is None:
                self.branchesToBeMigrated[str(repoID)] = data[repoID]["Branches"]
            else:
                self.branchesToBeMigrated[str(repoID)] = list(
                    set(data[repoID]["Branches"]).union(set(data[repoID]["StaleBranches"])))
            self.repoNames[str(repoID)] = data[repoID]["Name"]
            self.namespaces[str(repoID)] = data[repoID]["Namespace"]
        self.repoIDS = data.keys()