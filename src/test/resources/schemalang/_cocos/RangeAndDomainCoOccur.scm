/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema RangeAndDomainCoOccur {

    num_epoch: N1(10:20)<100, 200, 300> // this should not be possible
}