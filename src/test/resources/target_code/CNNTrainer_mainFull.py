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
        num_epoch=5,
        load_checkpoint=True,
        context='gpu',
        normalize=True,
        eval_metric='mse',
        optimizer='rmsprop',
        optimizer_params={
            'weight_decay': 0.01,
            'centered': True,
            'gamma2': 0.9,
            'gamma1': 0.9,
            'clip_weights': 10.0,
            'learning_rate_decay': 0.9,
            'epsilon': 1.0E-6,
            'rescale_grad': 1.1,
            'clip_gradient': 10.0,
            'learning_rate_minimum': 1.0E-5,
            'learning_rate_policy': 'step',
            'learning_rate': 0.001,
            'step_size': 1000}
    )
    main_net2 = CNNCreator_main_net2.CNNCreator_main_net2()
    main_net2.train(
        batch_size=100,
        num_epoch=10,
        load_checkpoint=False,
        context='gpu',
        normalize=False,
        eval_metric='topKAccuracy',
        optimizer='adam',
        optimizer_params={
            'epsilon': 1.0E-6,
            'weight_decay': 0.01,
            'rescale_grad': 1.1,
            'beta1': 0.9,
            'clip_gradient': 10.0,
            'beta2': 0.9,
            'learning_rate_minimum': 0.001,
            'learning_rate_policy': 'exp',
            'learning_rate': 0.001,
            'learning_rate_decay': 0.9,
            'step_size': 1000}
    )
