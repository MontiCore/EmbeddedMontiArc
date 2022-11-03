# (c) https://github.com/MontiCore/monticore
import logging
import pathlib

import mxnet as mx

import CNNCreator_fullConfig
import CNNDataLoader_fullConfig
import CNNSupervisedTrainer_fullConfig
from CNNDatasets_fullConfig import RetrainingConf

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    fullConfig_creator = CNNCreator_fullConfig.CNNCreator_fullConfig()
    fullConfig_creator.validate_parameters()
    fullConfig_loader = CNNDataLoader_fullConfig.CNNDataLoader_fullConfig()
    fullConfig_trainer = CNNSupervisedTrainer_fullConfig.CNNSupervisedTrainer_fullConfig(
        fullConfig_loader,
        fullConfig_creator
    )

    prev_dataset = None
    retraining_conf = fullConfig_loader.load_retraining_conf()
    for dataset in retraining_conf.changes:
        fullConfig_creator.dataset = dataset
        if(dataset.retraining):
            if prev_dataset: 
                logger.info("Retrain dataset %s on top of dataset %s.", dataset.id, prev_dataset.id)
            else: 
                logger.info("Dataset %s needs to be trained. Hash was different during the last EMADL2CPP run.", dataset.id)

            optimizer = 'rmsprop'
            optimizer_params = {
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



            fullConfig_trainer.train(
                dataset=dataset,
                test_dataset=retraining_conf.testing,
                batch_size=100,
                num_epoch=5,
                load_checkpoint=True,
                load_pretrained=bool(prev_dataset),
                load_pretrained_dataset=prev_dataset,
                context='gpu',
                preprocessing=False,
                normalize=True,
                eval_metric='mse',
                eval_metric_params={
                },
                loss='softmax_cross_entropy',
                    loss_params={
                        'sparse_label': True,
                        'loss_axis': -1,
                        'from_logits': False        },
            optimizer=optimizer,
            optimizer_params=optimizer_params,
            )
        else: 
            logger.info("Skipped training of dataset %s. Training is not necessary", dataset.id)
        
        prev_dataset = dataset

