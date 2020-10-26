import logging
import mxnet as mx
import CNNCreator_memorieswithproductkeys_network
import CNNDataLoader_memorieswithproductkeys_network
import CNNSupervisedTrainer_memorieswithproductkeys_network

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    memorieswithproductkeys_network_creator = CNNCreator_memorieswithproductkeys_network.CNNCreator_memorieswithproductkeys_network()
    memorieswithproductkeys_network_loader = CNNDataLoader_memorieswithproductkeys_network.CNNDataLoader_memorieswithproductkeys_network()
    memorieswithproductkeys_network_trainer = CNNSupervisedTrainer_memorieswithproductkeys_network.CNNSupervisedTrainer_memorieswithproductkeys_network(
        memorieswithproductkeys_network_loader,
        memorieswithproductkeys_network_creator
    )

    memorieswithproductkeys_network_trainer.train(
        batch_size=4,
        num_epoch=1,
        load_checkpoint=False,
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
