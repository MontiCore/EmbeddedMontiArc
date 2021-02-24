/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema RangeAndDomainDontCoOccur {

    num_epoch: N1(10:20)
    batch_size: N1<100, 200, 300>
}