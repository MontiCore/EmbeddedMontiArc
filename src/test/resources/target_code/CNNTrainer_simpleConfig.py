import logging
import mxnet as mx
import supervised_trainer
import CNNCreator_simpleConfig
import CNNDataLoader_simpleConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    simpleConfig_creator = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig_loader = CNNDataLoader_simpleConfig.simpleConfigDataLoader()
    simpleConfig_trainer = supervised_trainer.CNNSupervisedTrainer(simpleConfig_loader,
        simpleConfig_creator)

    simpleConfig_trainer.train(
        batch_size=100,
        num_epoch=50,
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001}
    )
