/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema DefaultValueIsNotInRange {

    closed_interval_min = -3: Z[-2:2]
    closed_interval_max = 3: Z[-2:2]

    open_interval_min = -2: Z(-2:2)
    open_interval_max = 2: Z(-2:2)

    rightopen_interval_min = -3: Z[-2:2)
    rightopen_interval_max = 2: Z[-2:2)

    leftopen_interval_min = -2: Z(-2:2]
    leftopen_interval_max = 3: Z(-2:2]
}