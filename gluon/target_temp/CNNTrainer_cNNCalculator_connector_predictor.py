
import mxnet as mx
import logging
import os
import numpy as np
import time
import shutil
from mxnet import gluon, autograd, nd

import CNNCreator_cNNCalculator_connector_predictor
import CNNDataLoader_cNNCalculator_connector_predictor
import CNNGanTrainer_cNNCalculator_connector_predictor

from gan.CNNCreator_cNNCalculator_discriminator import CNNCreator_cNNCalculator_discriminator

if __name__ == "__main__":

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    data_loader = CNNDataLoader_cNNCalculator_connector_predictor.CNNDataLoader_cNNCalculator_connector_predictor()

    gen_creator = CNNCreator_cNNCalculator_connector_predictor.CNNCreator_cNNCalculator_connector_predictor()
    dis_creator = CNNCreator_cNNCalculator_discriminator()

    cNNCalculator_connector_predictor_trainer = CNNGanTrainer_cNNCalculator_connector_predictor.CNNGanTrainer_cNNCalculator_connector_predictor(
        data_loader,
        gen_creator,
        dis_creator,
    )

    cNNCalculator_connector_predictor_trainer.train(
        batch_size=64,
        num_epoch=1,
        load_checkpoint=False,
        context='cpu',
        normalize=False,
        loss ='sigmoid_binary_cross_entropy',
        optimizer='adam',
        optimizer_params={
            'beta1': 0.5,
            'learning_rate': 2.0E-4        },
        noise_distribution = 'gaussian',
        noise_distribution_params = {
            'mean_value': 0,
            'spread_value': 1
        })


