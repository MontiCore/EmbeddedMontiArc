#ifndef TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_C21_TEST
#define TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_C21_TEST

#include "catch.hpp"
#include "../testing_subpackage1_mySuperAwesomeComponent1_c21.h"

TEST_CASE("testing.subpackage1.Comp2Test1", "[testing_subpackage1_mySuperAwesomeComponent1_c21]") {
    testing_subpackage1_mySuperAwesomeComponent1_c21 component;
    component.init();
            component.in2 = 10.1;
            component.in1 = 12.3;
        component.execute();
    REQUIRE( component.out1 >= 59.6 );
    REQUIRE( component.out1 <= 59.800000000000004 );
            component.in2 = 12.1;
            component.in1 = 45.6;
        component.execute();
    REQUIRE( component.out1 >= 93.30000000000001 );
    REQUIRE( component.out1 <= 93.5 );
            component.in2 = 14.1;
            component.in1 = 78.9;
        component.execute();
    REQUIRE( component.out1 >= 96.80000000000001 );
    REQUIRE( component.out1 <= 97.0 );
}
TEST_CASE("testing.subpackage1.Comp2Test3", "[testing_subpackage1_mySuperAwesomeComponent1_c21]") {
    testing_subpackage1_mySuperAwesomeComponent1_c21 component;
    component.init();
            component.in2 = 6.0;
            component.in1 = 1.0;
        component.execute();
    REQUIRE( component.out1 >= 97.10000000000001 );
    REQUIRE( component.out1 <= 97.3 );
            component.in2 = 7.0;
            component.in1 = 2.0;
        component.execute();
    REQUIRE( component.out1 >= 92.30000000000001 );
    REQUIRE( component.out1 <= 92.5 );
            component.in2 = 8.0;
            component.in1 = 3.0;
        component.execute();
    REQUIRE( component.out1 >= 87.5 );
    REQUIRE( component.out1 <= 87.69999999999999 );
            component.in2 = 9.0;
            component.in1 = 4.0;
        component.execute();
    REQUIRE( component.out1 >= 83.4 );
    REQUIRE( component.out1 <= 83.6 );
            component.in2 = 1.0;
            component.in1 = 5.0;
        component.execute();
    REQUIRE( component.out1 >= 96.0 );
    REQUIRE( component.out1 <= 96.19999999999999 );
}
TEST_CASE("testing.subpackage1.Comp2Test2", "[testing_subpackage1_mySuperAwesomeComponent1_c21]") {
    testing_subpackage1_mySuperAwesomeComponent1_c21 component;
    component.init();
            component.in2 = 1.1;
            component.in1 = 1.1;
        component.execute();
    REQUIRE( component.out1 >= 49.9 );
    REQUIRE( component.out1 <= 50.1 );
}

#endif

