# (c) https://github.com/MontiCore/monticore
import logging
import CNNCreator_simpleConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)


    simpleConfig = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig.train(
        batch_size=100,
        num_epoch=50,
        loss='cross_entropy',
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001},
    )
