# (c) https://github.com/MontiCore/monticore
import logging
import CNNCreator_emptyConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)


    emptyConfig = CNNCreator_emptyConfig.CNNCreator_emptyConfig()
    emptyConfig.train(
    )
