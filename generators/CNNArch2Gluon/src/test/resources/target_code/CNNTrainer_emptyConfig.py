# (c) https://github.com/MontiCore/monticore
import logging
import pathlib

import mxnet as mx

import CNNCreator_emptyConfig
import CNNDataLoader_emptyConfig
from CNNDatasets_emptyConfig import RetrainingConf
import CNNDataCleaner_emptyConfig
import CNNSupervisedTrainer_emptyConfig

if __name__ == "__main__":
    logger = logging.getLogger()
    logging.basicConfig(level=logging.DEBUG)
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    emptyConfig_creator = CNNCreator_emptyConfig.CNNCreator_emptyConfig()
    emptyConfig_creator.validate_parameters()
    emptyConfig_cleaner = CNNDataCleaner_emptyConfig.CNNDataCleaner_emptyConfig()
    emptyConfig_loader = CNNDataLoader_emptyConfig.CNNDataLoader_emptyConfig(
        emptyConfig_cleaner
    )

    prev_dataset = None
    retraining_conf = emptyConfig_loader.load_retraining_conf()
    
    emptyConfig_trainer = CNNSupervisedTrainer_emptyConfig.CNNSupervisedTrainer_emptyConfig(
        emptyConfig_loader,
        emptyConfig_creator
    )

    for dataset in retraining_conf.changes:
        emptyConfig_creator.dataset = dataset
        if(dataset.retraining):
            if prev_dataset: 
                logger.info("Retrain dataset %s on top of dataset %s.", dataset.id, prev_dataset.id)
            else: 
                logger.info("Dataset %s needs to be trained. Hash was different during the last EMADL2CPP run.", dataset.id)




            emptyConfig_trainer.train(
                dataset=dataset,
                test_dataset=retraining_conf.testing,
                load_pretrained=bool(prev_dataset),
                load_pretrained_dataset=prev_dataset,
                preprocessing=False,
            optimizer=optimizer,
            optimizer_params=optimizer_params,
            )
        else: 
            logger.info("Skipped training of dataset %s. Training is not necessary", dataset.id)

        prev_dataset = dataset

