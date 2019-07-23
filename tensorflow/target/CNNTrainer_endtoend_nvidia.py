import logging
import CNNCreator_endtoend_nvidia

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    endtoend_nvidia = CNNCreator_endtoend_nvidia.CNNCreator_endtoend_nvidia()
    endtoend_nvidia.train(
        batch_size=2,
        num_epoch=5,
        load_checkpoint=False,
        context='cpu',
        normalize=False,
        loss='cross_entropy',
        optimizer='adam',
        optimizer_params={
}
    )
