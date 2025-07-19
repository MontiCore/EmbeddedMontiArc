# (c) https://github.com/MontiCore/monticore
import logging
import mxnet as mx
import CNNCreator_emptyConfig
import CNNDataCleaner_emptyConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig_cleaner = CNNDataCleaner_emptyConfig.CNNDataCleaner_emptyConfig()
    emptyConfig = CNNCreator_emptyConfig.CNNCreator_emptyConfig(
        emptyConfig_cleaner
    )
    emptyConfig.train(
    )
