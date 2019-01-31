#ifndef TESTA_CALCULATOR_ARGS1_TEST
#define TESTA_CALCULATOR_ARGS1_TEST

#include "catch.hpp"
#include "../TestA_calculator_args1.h"

TEST_CASE("TestA.ArgmaxStream", "[TestA_calculator_args1]") {
    TestA_calculator_args1 component;
    component.init();
        component.execute();
    REQUIRE( component.max >= 100.0 );
    REQUIRE( component.max <= 100.0 );
    std::cout << "TestA.ArgmaxStream: success\n";
}

#endif

