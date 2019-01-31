#ifndef TESTA_ADD_TEST
#define TESTA_ADD_TEST

#include "catch.hpp"
#include "../TestA_add.h"

TEST_CASE("TestA.AddStream", "[TestA_add]") {
    TestA_add component;
    component.init();
            component.num1 = 1.0;
            component.num2 = 2.0;
        component.execute();
    REQUIRE( component.sum >= 3.0 );
    REQUIRE( component.sum <= 3.0 );
    std::cout << "TestA.AddStream: success\n";
}

#endif

