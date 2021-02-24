/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema ValidRequiresRules {
    num_epoch = 100000: N1
    batch_size = 64: N1

    num_epoch requires batch_size
    batch_size requires num_epoch
}