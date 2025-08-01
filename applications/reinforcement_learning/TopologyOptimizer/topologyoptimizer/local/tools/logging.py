import os
from pathlib import Path
from sys import stdout
from config import config
import logging

def initialize_logging():
    log_folder = Path(config.get('DEFAULT', 'LocalLogFolder'))
    logger = logging.getLogger()
    logging.getLogger('paramiko').setLevel(logging.INFO)
    logger.setLevel(logging.INFO)
    if not log_folder.exists():
        log_folder.mkdir()
    file_handler = logging.FileHandler(log_folder / 'topologyoptimizer.log')
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    file_handler.setLevel(logging.DEBUG)
    stdout_handler = logging.StreamHandler(stdout)
    stdout_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
    logger.addHandler(stdout_handler)