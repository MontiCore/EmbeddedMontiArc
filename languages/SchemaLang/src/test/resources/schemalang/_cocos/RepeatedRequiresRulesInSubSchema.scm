/* (c) https://github.com/MontiCore/monticore */
// package de.monticore.lang.schemalang;

schema RepeatedRequiresRulesInSubSchema extends RepeatedRequiresRulesInSubSchemaSuper {
    noise_input requires num_epoch
    normalize requires batch_size
}