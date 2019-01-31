#ifndef TESTA_ARGMAXS_TEST
#define TESTA_ARGMAXS_TEST

#include "catch.hpp"
#include "../TestA_argmaxs.h"

TEST_CASE("TestA.ArgmaxStream", "[TestA_argmaxs]") {
    TestA_argmaxs component;
    component.init();
        component.execute();
    REQUIRE( component.max >= 100.0 );
    REQUIRE( component.max <= 100.0 );
    std::cout << "TestA.ArgmaxStream: success\n";
}

#endif

