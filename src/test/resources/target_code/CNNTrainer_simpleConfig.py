# (c) https://github.com/MontiCore/monticore
import logging
import pathlib

import mxnet as mx

import CNNCreator_simpleConfig
import CNNDataLoader_simpleConfig
from CNNDatasets_simpleConfig import RetrainingConf
import CNNDataCleaner_simpleConfig
import CNNSupervisedTrainer_simpleConfig

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    simpleConfig_creator = CNNCreator_simpleConfig.CNNCreator_simpleConfig()
    simpleConfig_creator.validate_parameters()
    simpleConfig_cleaner = CNNDataCleaner_simpleConfig.CNNDataCleaner_simpleConfig()
    simpleConfig_loader = CNNDataLoader_simpleConfig.CNNDataLoader_simpleConfig(
        simpleConfig_cleaner
    )

    prev_dataset = None
    retraining_conf = simpleConfig_loader.load_retraining_conf()
    
    simpleConfig_trainer = CNNSupervisedTrainer_simpleConfig.CNNSupervisedTrainer_simpleConfig(
        simpleConfig_loader,
        simpleConfig_creator
    )

    for dataset in retraining_conf.changes:
        simpleConfig_creator.dataset = dataset
        if(dataset.retraining):
            if prev_dataset: 
                logger.info("Retrain dataset %s on top of dataset %s.", dataset.id, prev_dataset.id)
            else: 
                logger.info("Dataset %s needs to be trained. Hash was different during the last EMADL2CPP run.", dataset.id)

            optimizer = 'adam'
            optimizer_params = {
                'learning_rate': 0.001}



            simpleConfig_trainer.train(
                dataset=dataset,
                test_dataset=retraining_conf.testing,
                batch_size=100,
                num_epoch=50,
                load_pretrained=bool(prev_dataset),
                load_pretrained_dataset=prev_dataset,
                preprocessing=False,
                loss='cross_entropy',
            optimizer=optimizer,
            optimizer_params=optimizer_params,
            )
        else: 
            logger.info("Skipped training of dataset %s. Training is not necessary", dataset.id)

        prev_dataset = dataset

