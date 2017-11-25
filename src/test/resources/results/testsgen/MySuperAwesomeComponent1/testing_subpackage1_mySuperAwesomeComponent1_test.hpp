#ifndef TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_TEST
#define TESTING_SUBPACKAGE1_MYSUPERAWESOMECOMPONENT1_TEST

#include "catch.hpp"
#include "../testing_subpackage1_mySuperAwesomeComponent1.h"

TEST_CASE("testing.subpackage1.MySuperAwesomeComponent1", "[testing_subpackage1_mySuperAwesomeComponent1]") {
    testing_subpackage1_mySuperAwesomeComponent1 component;
    component.init();
            component.in2 = 0.0;
            component.in1 = true;
            component.in3 = 0.0;
        component.execute();
    REQUIRE( component.out2 >= 0.0 );
    REQUIRE( component.out2 <= 0.0 );
    REQUIRE( component.out3 >= 100.0 );
    REQUIRE( component.out3 <= 100.0 );
    REQUIRE( component.out4 >= 99.9 );
    REQUIRE( component.out4 <= 100.1 );
        REQUIRE_FALSE( component.out1 );
            component.in2 = 1.0;
            component.in1 = false;
            component.in3 = 12.3;
        component.execute();
    REQUIRE( component.out2 >= -1.0 );
    REQUIRE( component.out2 <= -1.0 );
    REQUIRE( component.out3 >= 87.69 );
    REQUIRE( component.out3 <= 87.71000000000001 );
    REQUIRE( component.out4 >= 97.9 );
    REQUIRE( component.out4 <= 98.1 );
        REQUIRE( component.out1 );
            component.in2 = 2.0;
            component.in1 = true;
            component.in3 = 100.0;
        component.execute();
    REQUIRE( component.out2 >= -2.0 );
    REQUIRE( component.out2 <= -2.0 );
    REQUIRE( component.out3 >= 0.0 );
    REQUIRE( component.out3 <= 0.0 );
    REQUIRE( component.out4 >= 99.9 );
    REQUIRE( component.out4 <= 100.1 );
        REQUIRE_FALSE( component.out1 );
}

#endif

