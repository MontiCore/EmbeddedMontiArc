#ifndef TESTA_CALCULATOR_TEST
#define TESTA_CALCULATOR_TEST

#include "catch.hpp"
#include "../TestA_calculator.h"

TEST_CASE("TestA.CalculatorStream", "[TestA_calculator]") {
    TestA_calculator component;
    component.init();
        component.execute();
    std::cout << "TestA.CalculatorStream: success\n";
}

#endif

