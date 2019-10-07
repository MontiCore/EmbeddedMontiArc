import logging
import mxnet as mx
import CNNCreator_simpleConfig
import CNNDataLoader_simpleConfig
import CNNSupervisedTrainer_simpleConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    simpleConfig_creator = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig_loader = CNNDataLoader_simpleConfig.CNNDataLoader_simpleConfig()
    simpleConfig_trainer = CNNSupervisedTrainer_simpleConfig.CNNSupervisedTrainer_simpleConfig(
        simpleConfig_loader,
        simpleConfig_creator
    )

    simpleConfig_trainer.train(
        batch_size=100,
        num_epoch=50,
        loss='cross_entropy',
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001}
    )
