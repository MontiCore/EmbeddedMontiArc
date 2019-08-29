/* (c) https://github.com/MontiCore/monticore */
#pragma once

#ifndef tests_a_compA_h
#define tests_a_compA_h

#include <iostream>

using namespace std;

class tests_a_compA {

public:
    tests_a_compA(double value);
    double mqttInQ = 0;
    double mqttOutQ = 0;
    int mqttInN = 0;
    int mqttOutN = 0;
    int mqttInZ = 0;
    int mqttOutZ = 0;
    bool mqttInB = false;
    bool mqttOutB = false;
};

#endif /* tests_a_compA_h */
