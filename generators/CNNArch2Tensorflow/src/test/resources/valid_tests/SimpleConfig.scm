schema SimpleConfig {
    num_epoch: N0
    batch_size: N0

    loss: loss_type
    optimizer: optimizer_type

    loss_type {
        values:
            cross_entropy;
    }

    optimizer_type {
        values:
            adam;

        define adam {
            learning_rate: Q
        }
    }
}