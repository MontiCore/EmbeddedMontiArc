/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema SchemaWithoutReferenceCycle extends SuperSchema {
    num_epoch = 100000: N1
    load_checkpoint: B
}