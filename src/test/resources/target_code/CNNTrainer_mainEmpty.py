import logging
import mxnet as mx
import CNNCreator_main_net1

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    main_net1 = CNNCreator_main_net1.CNNCreator_main_net1()
    main_net1.train(
    )
