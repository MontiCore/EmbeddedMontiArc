/* (c) https://github.com/MontiCore/monticore */
#include <stdio.h>
#include "tests_a_compA.h"

tests_a_compA::tests_a_compA(double value)
{
    mqttOutQ = value;
    mqttOutN = (int) value;
    mqttOutZ = (int) value;
    mqttOutB = (value >= 1);
}
