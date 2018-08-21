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
        batch_size=64,
        num_epoch=10,
        load_checkpoint=False,
        context='gpu',
        normalize=True,
        optimizer='adam',
        optimizer_params={
            'weight_decay': 1.0E-4,
            'learning_rate': 0.01,
            'learning_rate_decay': 0.8,
            'step_size': 1000}
    )
    main_net2 = CNNCreator_main_net2.CNNCreator_main_net2()
    main_net2.train(
        batch_size=32,
        num_epoch=10,
        load_checkpoint=False,
        context='gpu',
        normalize=True,
        optimizer='adam',
        optimizer_params={
            'weight_decay': 1.0E-4,
            'learning_rate': 0.01,
            'learning_rate_decay': 0.8,
            'step_size': 1000}
    )
