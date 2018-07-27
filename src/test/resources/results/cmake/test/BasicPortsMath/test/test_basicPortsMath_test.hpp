#ifndef TEST_BASICPORTSMATH_TEST
#define TEST_BASICPORTSMATH_TEST

#include "catch.hpp"
#include "../test_basicPortsMath.h"

TEST_CASE("test.BasicPortsMath", "[test_basicPortsMath]") {
    test_basicPortsMath component;
    component.init();
            component.counter = -10.0;
        component.execute();
    REQUIRE( component.result >= 0.0 );
    REQUIRE( component.result <= 0.0 );
            component.counter = -1.0;
        component.execute();
    REQUIRE( component.result >= 0.0 );
    REQUIRE( component.result <= 0.0 );
            component.counter = 0.0;
        component.execute();
    REQUIRE( component.result >= 0.0 );
    REQUIRE( component.result <= 0.0 );
            component.counter = 1.0;
        component.execute();
    REQUIRE( component.result >= 1.0 );
    REQUIRE( component.result <= 1.0 );
            component.counter = 100.0;
        component.execute();
    REQUIRE( component.result >= 100.0 );
    REQUIRE( component.result <= 100.0 );
            component.counter = 1000.0;
        component.execute();
    REQUIRE( component.result >= 100.0 );
    REQUIRE( component.result <= 100.0 );
    std::cout << "test.BasicPortsMath: success\n";
}

#endif

