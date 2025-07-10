/* (c) https://github.com/MontiCore/monticore */

schema SimpleConfig {
     num_epoch: N0
     batch_size: N0
     optimizer: optimizer_type

     optimizer_type {
        values:
            adam;

        define adam {
            learning_rate: Q
        }
     }
}