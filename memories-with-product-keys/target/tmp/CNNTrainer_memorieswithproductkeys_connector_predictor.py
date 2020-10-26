import logging
import mxnet as mx
import CNNCreator_memorieswithproductkeys_connector_predictor
import CNNDataLoader_memorieswithproductkeys_connector_predictor
import CNNSupervisedTrainer_memorieswithproductkeys_connector_predictor

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    memorieswithproductkeys_connector_predictor_creator = CNNCreator_memorieswithproductkeys_connector_predictor.CNNCreator_memorieswithproductkeys_connector_predictor()
    memorieswithproductkeys_connector_predictor_loader = CNNDataLoader_memorieswithproductkeys_connector_predictor.CNNDataLoader_memorieswithproductkeys_connector_predictor()
    memorieswithproductkeys_connector_predictor_trainer = CNNSupervisedTrainer_memorieswithproductkeys_connector_predictor.CNNSupervisedTrainer_memorieswithproductkeys_connector_predictor(
        memorieswithproductkeys_connector_predictor_loader,
        memorieswithproductkeys_connector_predictor_creator
    )

    memorieswithproductkeys_connector_predictor_trainer.train(
        batch_size=5,
        num_epoch=1,
        load_checkpoint=False,
        log_period=10,
        context='cpu',
        preprocessing=False,
        normalize=False,
        shuffle_data=True,
        loss='cross_entropy',
        optimizer='adam',
        optimizer_params={
            'weight_decay': 0.01,
            'learning_rate': 3.0E-5}
    )
