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