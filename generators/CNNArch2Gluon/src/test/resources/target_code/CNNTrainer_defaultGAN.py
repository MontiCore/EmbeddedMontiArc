# (c) https://github.com/MontiCore/monticore

import mxnet as mx
import logging
import os
import numpy as np
import time
import shutil
from mxnet import gluon, autograd, nd

import CNNCreator_defaultGAN
import CNNDataLoader_defaultGAN
import CNNCreator_defaultGAN
import CNNGanTrainer_defaultGAN
from gan.CNNCreator_Discriminator import CNNCreator_Discriminator



if __name__ == "__main__":

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)
    logging.getLogger('matplotlib.font_manager').disabled = True
    logging.getLogger('matplotlib.colorbar').disabled = True
    logger.addHandler(handler)

    data_loader = CNNDataLoader_defaultGAN.CNNDataLoader_defaultGAN()
    gen_creator = CNNCreator_defaultGAN.CNNCreator_defaultGAN()
    dis_creator = CNNCreator_Discriminator()

    defaultGAN_trainer = CNNGanTrainer_defaultGAN.CNNGanTrainer_defaultGAN(
        data_loader,
        gen_creator,
        dis_creator,
    )

    defaultGAN_trainer.train(
        batch_size=64,
        num_epoch=10,
        load_checkpoint=False,
        context='cpu',
        normalize=False,
        preprocessing=False,
        optimizer='adam',
        optimizer_params={
            'beta1': 0.5,
            'learning_rate': 2.0E-4        },
        discriminator_optimizer= 'adam',
        discriminator_optimizer_params= {
            'beta1': 0.5,
            'learning_rate': 2.0E-4},
        noise_distribution = 'gaussian',
        noise_distribution_params = {
            'mean_value': 0,
            'spread_value': 1
     },
        k_value=1,
        noise_input="noise",
        gen_loss_weight=0.5,
        dis_loss_weight=0.5,
        log_period=10,
        print_images=True,
)
