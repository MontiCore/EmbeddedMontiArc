import logging
import mxnet as mx
import supervised_trainer
import CNNCreator_emptyConfig
import CNNDataLoader_emptyConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig_creator = CNNCreator_emptyConfig.CNNCreator_emptyConfig()
    emptyConfig_loader = CNNDataLoader_emptyConfig.emptyConfigDataLoader()
    emptyConfig_trainer = supervised_trainer.CNNSupervisedTrainer(emptyConfig_loader,
        emptyConfig_creator)

    emptyConfig_trainer.train(
    )
