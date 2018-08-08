#ifndef TEST_MATH_FLOATDIVISIONTEST_TEST
#define TEST_MATH_FLOATDIVISIONTEST_TEST

#include "catch.hpp"
#include "../test_math_floatDivisionTest.h"

TEST_CASE("test.math.FloatDivisionTest", "[test_math_floatDivisionTest]") {
    test_math_floatDivisionTest component;
    component.init();
        component.execute();
    REQUIRE( component.out1 >= 0.4999 );
    REQUIRE( component.out1 <= 0.5001 );
    std::cout << "test.math.FloatDivisionTest: success\n";
}

#endif

