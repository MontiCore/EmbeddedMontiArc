<#include "/Common.ftl">
#ifndef TESTS_MAIN
#define TESTS_MAIN

//#define CATCH_CONFIG_RUNNER
#define CATCH_CONFIG_FAST_COMPILE
#include "catch.hpp"
/*
int main(int argc, char* argv[]) {
    Catch::Session session;
    int returnCode = session.applyCommandLine(argc, argv);
    if (returnCode != 0) {
        return returnCode;
    }
    int numFailed = session.run();
    return numFailed;
}*/

<#list viewModel.includes as i>
#include "${i}"
</#list>

#endif
