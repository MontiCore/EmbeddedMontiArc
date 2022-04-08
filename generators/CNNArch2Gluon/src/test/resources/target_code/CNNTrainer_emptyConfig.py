# (c) https://github.com/MontiCore/monticore
import logging
import mxnet as mx

import CNNCreator_emptyConfig
import CNNDataLoader_emptyConfig
import CNNSupervisedTrainer_emptyConfig

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig_creator = CNNCreator_emptyConfig.CNNCreator_emptyConfig()
    emptyConfig_creator.validate_parameters()
    emptyConfig_loader = CNNDataLoader_emptyConfig.CNNDataLoader_emptyConfig()
    emptyConfig_trainer = CNNSupervisedTrainer_emptyConfig.CNNSupervisedTrainer_emptyConfig(
        emptyConfig_loader,
        emptyConfig_creator
    )

    emptyConfig_trainer.train(
        preprocessing=False,
    )
