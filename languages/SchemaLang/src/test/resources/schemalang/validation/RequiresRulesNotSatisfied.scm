/* (c) https://github.com/MontiCore/monticore */

schema RequiresRulesNotSatisfied {

    batch_size: N1
    num_epoch: N1
    normalize: B

    batch_size requires num_epoch
    normalize requires batch_size, num_epoch
}