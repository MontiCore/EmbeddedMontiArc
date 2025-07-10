/* (c) https://github.com/MontiCore/monticore */

schema RequiresRulesSatisfied {

    batch_size: N1
    num_epoch: N1
    normalize: B
    use_teacher_forcing: B

    batch_size requires num_epoch
    num_epoch requires normalize, use_teacher_forcing
}