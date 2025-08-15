# (c) https://github.com/MontiCore/monticore
from caffe2.python import workspace, core, model_helper, brew, optimizer
from caffe2.python.predictor import mobile_exporter
from caffe2.proto import caffe2_pb2

import numpy as np
import logging
import CNNCreator_fullConfig
import CNNDataCleaner_fullConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    fullConfig_cleaner = CNNDataCleaner_fullConfig.CNNDataCleaner_fullConfig()
    fullConfig = CNNCreator_fullConfig.CNNCreator_fullConfig(
        fullConfig_cleaner
    )
    fullConfig.train(
        num_epoch=5,
        batch_size=100,
        context='gpu',
        eval_metric='mse',
        opt_type='rmsprop',
            epsilon=1.0E-6,
            weight_decay=0.01,
            gamma1=0.9,
            policy='step',
            base_learning_rate=0.001,
            gamma=0.9,
            stepsize=1000
    )
