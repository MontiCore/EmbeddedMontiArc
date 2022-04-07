# (c) https://github.com/MontiCore/monticore
import logging
import mxnet as mx

import CNNCreator_simpleConfig
import CNNDataLoader_simpleConfig
import CNNSupervisedTrainer_simpleConfig

if __name__ == "__main__":
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    simpleConfig_creator = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig_creator.validate_parameters()
    simpleConfig_loader = CNNDataLoader_simpleConfig.CNNDataLoader_simpleConfig()
    simpleConfig_trainer = CNNSupervisedTrainer_simpleConfig.CNNSupervisedTrainer_simpleConfig(
        simpleConfig_loader,
        simpleConfig_creator
    )

    simpleConfig_trainer.train(
        batch_size=100,
        num_epoch=50,
        preprocessing=False,
        loss='cross_entropy',
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001},
    )
