# (c) https://github.com/MontiCore/monticore

import mxnet as mx
import logging
import os
import numpy as np
import time
import shutil
from mxnet import gluon, autograd, nd

import CNNCreator_vQDecoder
import CNNDataLoader_vQDecoder
import CNNAutoencoderTrainer_VQEncoder
from CNNCreator_VQEncoder import CNNCreator_VQEncoder



if __name__ == "__main__":

    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)
    logging.getLogger('matplotlib.font_manager').disabled = True
    logging.getLogger('matplotlib.colorbar').disabled = True
    logger.addHandler(handler)

    data_loader = CNNDataLoader_vQDecoder.CNNDataLoader_vQDecoder()
    encoder_creator = CNNCreator_VQEncoder()
    decoder_creator = CNNCreator_vQDecoder.CNNCreator_vQDecoder()
    vQDecoder_trainer = CNNAutoencoderTrainer_VQEncoder.CNNAutoencoderTrainer(
    data_loader,
    encoder_creator,
    decoder_creator,
    )

    vQDecoder_trainer.train(
        batch_size=200,
        num_epoch=15,
        checkpoint_period=5,
        context='gpu',
        normalize=False,
        preprocessing=False,
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001        },
        reconstruction_loss='mse',
        kl_loss_weight=1,
        print_images=True,
)
