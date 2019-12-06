from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_unsupportedConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    unsupportedConfig = CNNCreator_unsupportedConfig.CNNCreator_unsupportedConfig()
    unsupportedConfig.train(
        num_epoch=5,
        batch_size=100,
        context='gpu',

    )
