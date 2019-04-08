import logging
import mxnet as mx
import supervised_trainer
import CNNCreator_fullConfig
import CNNDataLoader_fullConfig

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    fullConfig_creator = CNNCreator_fullConfig.CNNCreator_fullConfig()
    fullConfig_loader = CNNDataLoader_fullConfig.fullConfigDataLoader()
    fullConfig_trainer = supervised_trainer.CNNSupervisedTrainer(fullConfig_loader,
        fullConfig_creator)

    fullConfig_trainer.train(
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
