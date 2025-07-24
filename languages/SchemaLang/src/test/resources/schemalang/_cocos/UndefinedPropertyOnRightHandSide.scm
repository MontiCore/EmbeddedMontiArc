/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema UndefinedPropertyOnRightHandSide {
    num_epoch = 100000: N1
    batch_size = 64: N1

    num_epoch requires noise_input,batch_size
    batch_size requires normalize,optimizer
}