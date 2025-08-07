/* (c) https://github.com/MontiCore/monticore */

schema Ranges {

    closed_interval: Z[-2:2]
    closed_interval_min: Z[-2:2]
    closed_interval_max: Z[-2:2]

    open_interval: Z(-2:2)
    open_interval_min: Z(-2:2)
    open_interval_max: Z(-2:2)

    rightopen_interval: Z[-2:2)
    rightopen_interval_min: Z[-2:2)
    rightopen_interval_max: Z[-2:2)

    leftopen_interval: Z(-2:2]
    leftopen_interval_min: Z(-2:2]
    leftopen_interval_max: Z(-2:2]
}