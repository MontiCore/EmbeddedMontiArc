import logging

file_handler = logging.FileHandler("output.log")
file_handler.setLevel(logging.INFO)
file_handler.setFormatter(logging.Formatter("%(asctime)s - %(levelname)s - %(message)s", datefmt='%H:%M:%S'))
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)
logger.handlers = []  # Remove if you want to see logs in console
logger.addHandler(file_handler)
print(logger.name)
