from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_simpleConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    simpleConfig = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig.train(
        num_epoch=50,
        batch_size=100,
        opt_type='adam',
        base_learning_rate=0.001
    )
