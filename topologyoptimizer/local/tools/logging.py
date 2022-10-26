from config import config
import logging

def initialize_logging():
    log_folder = config.get('DEFAULT', 'LocalLogFolder')
    logger = logging.getLogger()
    logging.getLogger('paramiko').setLevel(logging.INFO)
    logger.setLevel(logging.DEBUG)
    file_handler = logging.FileHandler(f'{log_folder}/topologyoptimizer.log')
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)