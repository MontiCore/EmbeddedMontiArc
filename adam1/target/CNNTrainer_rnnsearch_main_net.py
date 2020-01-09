import logging
import mxnet as mx
import CNNCreator_rnnsearch_main_net
import CNNDataLoader_rnnsearch_main_net
import CNNSupervisedTrainer_rnnsearch_main_net

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)
    logger = logging.getLogger()
    handler = logging.FileHandler("train.log", "w", encoding=None, delay="true")
    logger.addHandler(handler)

    rnnsearch_main_net_creator = CNNCreator_rnnsearch_main_net.CNNCreator_rnnsearch_main_net()
    rnnsearch_main_net_loader = CNNDataLoader_rnnsearch_main_net.CNNDataLoader_rnnsearch_main_net()
    rnnsearch_main_net_trainer = CNNSupervisedTrainer_rnnsearch_main_net.CNNSupervisedTrainer_rnnsearch_main_net(
        rnnsearch_main_net_loader,
        rnnsearch_main_net_creator
    )

    rnnsearch_main_net_trainer.train(
        batch_size=64,
        num_epoch=50,
        checkpoint_period=1,
        log_period=50,
        context='gpu',
        normalize=False,
        eval_metric='bleu',
        eval_metric_params={
            'exclude': [0, 2, 3],
        },
        eval_train=False,
        loss='softmax_cross_entropy_ignore_indices',
        loss_params={'ignore_indices': 0},
        optimizer='adam',
        optimizer_params={'learning_rate': 0.001, 'learning_rate_decay': 0.99, 'step_size': 1438}
    )
