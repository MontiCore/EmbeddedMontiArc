#ifndef TESTS_MAIN
#define TESTS_MAIN

#include "../Helper.h"
#define CATCH_CONFIG_RUNNER
#include "catch.hpp"

int main(int argc, char* argv[]) {
    Helper::init();
    Catch::Session session;
    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0) {
        octave_quit();
        return returnCode;
    }
    int numFailed = session.run();
    octave_quit();
    return numFailed;
}

#include "testing_subpackage1_mySuperAwesomeComponent1_test.hpp"
#include "testing_subpackage1_mySuperAwesomeComponent1_c11_test.hpp"
#include "testing_subpackage1_mySuperAwesomeComponent1_c21_test.hpp"

#endif
