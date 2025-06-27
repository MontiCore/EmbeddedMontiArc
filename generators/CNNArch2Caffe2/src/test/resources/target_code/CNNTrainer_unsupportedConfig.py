# (c) https://github.com/MontiCore/monticore
from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_unsupportedConfig
import CNNDataCleaner_unsupportedConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    unsupportedConfig_cleaner = CNNDataCleaner_unsupportedConfig.CNNDataCleaner_unsupportedConfig()
    unsupportedConfig = CNNCreator_unsupportedConfig.CNNCreator_unsupportedConfig(
        unsupportedConfig_cleaner
    )
    unsupportedConfig.train(
        num_epoch=5,
        batch_size=100,
        context='gpu',

    )
