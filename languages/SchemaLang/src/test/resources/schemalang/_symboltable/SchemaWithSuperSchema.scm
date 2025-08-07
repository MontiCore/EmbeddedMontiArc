/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema SchemaWithSuperSchema extends SuperSchema {
    num_epoch = 100000: N
    batch_size = 64: N
    load_checkpoint: B
}