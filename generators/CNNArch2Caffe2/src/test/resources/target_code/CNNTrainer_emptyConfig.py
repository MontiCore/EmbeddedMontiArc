# (c) https://github.com/MontiCore/monticore
from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_emptyConfig
import CNNDataCleaner_emptyConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig_cleaner = CNNDataCleaner_emptyConfig.CNNDataCleaner_emptyConfig()
    emptyConfig = CNNCreator_emptyConfig.CNNCreator_emptyConfig(
        emptyConfig_cleaner
    )
    emptyConfig.train(

    )
