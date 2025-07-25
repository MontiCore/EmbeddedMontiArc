/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema UndefinedPropertyOnLeftHandSide {
    num_epoch = 100000: N1
    batch_size = 64: N1

    noise_input requires num_epoch
    normalize requires batch_size
}