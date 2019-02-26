//created by TestsMainEntry.ftl
#ifndef TESTS_MAIN
#define TESTS_MAIN
#include <iostream>

#include "test_basicPortsMath_test.hpp"

int main(){
    std::cout << "=================Start stream testing=================" << std::endl;
    int errorCode = 0;
    errorCode += test_basicPortsMath_test::runTest();
    std::cout << "==================End stream testing==================" << std::endl;
    exit(errorCode);
}

#endif
