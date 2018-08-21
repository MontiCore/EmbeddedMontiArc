import logging
import mxnet as mx
import CNNCreator_main_net1
import CNNCreator_main_net2

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    main_net1 = CNNCreator_main_net1.CNNCreator_main_net1()
    main_net1.train(
        batch_size=100,
        num_epoch=50,
        optimizer='adam',
        optimizer_params={
            'learning_rate': 0.001}
    )
    main_net2 = CNNCreator_main_net2.CNNCreator_main_net2()
    main_net2.train(
        batch_size=100,
        num_epoch=5,
        optimizer='sgd',
        optimizer_params={
            'learning_rate': 0.1}
    )
