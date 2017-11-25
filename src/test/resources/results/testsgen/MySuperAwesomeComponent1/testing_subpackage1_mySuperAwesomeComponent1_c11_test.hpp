#ifndef TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_C11_TEST
#define TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_C11_TEST

#include "catch.hpp"
#include "../testing_subpackage1_mySuperAwesomeComponent1_c11.h"

TEST_CASE("testing.subpackage1.Comp1Test1", "[testing_subpackage1_mySuperAwesomeComponent1_c11]") {
    testing_subpackage1_mySuperAwesomeComponent1_c11 component;
    component.init();
            component.in2 = 0.0;
            component.in1 = true;
            component.in3 = 0.0;
        component.execute();
    REQUIRE( component.out2 >= 0.0 );
    REQUIRE( component.out2 <= 0.0 );
    REQUIRE( component.out3 >= 100.0 );
    REQUIRE( component.out3 <= 100.0 );
        REQUIRE_FALSE( component.out1 );
            component.in2 = 1.0;
            component.in1 = false;
            component.in3 = 12.3;
        component.execute();
    REQUIRE( component.out2 >= -1.0 );
    REQUIRE( component.out2 <= -1.0 );
    REQUIRE( component.out3 >= 87.69 );
    REQUIRE( component.out3 <= 87.71000000000001 );
        REQUIRE( component.out1 );
            component.in2 = 2.0;
            component.in1 = true;
            component.in3 = 100.0;
        component.execute();
    REQUIRE( component.out2 >= -2.0 );
    REQUIRE( component.out2 <= -2.0 );
    REQUIRE( component.out3 >= 0.0 );
    REQUIRE( component.out3 <= 0.0 );
        REQUIRE_FALSE( component.out1 );
}
TEST_CASE("testing.subpackage1.Comp1Test2", "[testing_subpackage1_mySuperAwesomeComponent1_c11]") {
    testing_subpackage1_mySuperAwesomeComponent1_c11 component;
    component.init();
            component.in2 = 0.0;
            component.in1 = false;
            component.in3 = 12.3;
        component.execute();
    REQUIRE( component.out2 >= 0.0 );
    REQUIRE( component.out2 <= 0.0 );
    REQUIRE( component.out3 >= 87.69 );
    REQUIRE( component.out3 <= 87.71000000000001 );
        REQUIRE( component.out1 );
            component.in2 = -11.0;
            component.in1 = true;
            component.in3 = 45.6;
        component.execute();
    REQUIRE( component.out2 >= 10.0 );
    REQUIRE( component.out2 <= 12.0 );
    REQUIRE( component.out3 >= 54.39 );
    REQUIRE( component.out3 <= 54.41 );
        REQUIRE_FALSE( component.out1 );
            component.in2 = -200.0;
            component.in1 = true;
            component.in3 = 78.9;
        component.execute();
    REQUIRE( component.out2 >= 200.0 );
    REQUIRE( component.out2 <= 200.0 );
    REQUIRE( component.out3 >= 21.09 );
    REQUIRE( component.out3 <= 21.110000000000003 );
        REQUIRE_FALSE( component.out1 );
}

#endif

