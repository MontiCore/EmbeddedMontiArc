/* (c) https://github.com/MontiCore/monticore */

schema UnsupportedConfig {

     context: enum {
        gpu;
     }

     num_epoch: N0
     batch_size: N0
     load_checkpoint: B
     normalize: B
     optimizer: optimizer_type

     optimizer_type {
        values:
            nag;

        define nag {
            learning_rate_minimum: Q
            rescale_grad: Q
            clip_gradient: Q
        }
     }
}