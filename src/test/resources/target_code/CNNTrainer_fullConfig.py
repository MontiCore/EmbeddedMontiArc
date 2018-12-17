from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_fullConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    fullConfig = CNNCreator_fullConfig.CNNCreator_fullConfig()
    fullConfig.train(
        num_epoch=5,
        batch_size=100,
        context='gpu',
        eval_metric='mse',
        opt_type='rmsprop',
        epsilon=1.0E-6,
        weight_decay=0.01,
        gamma=0.9,
        policy='step',
        base_learning_rate=0.001,
        stepsize=1000
    )
