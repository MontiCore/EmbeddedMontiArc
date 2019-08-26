/* (c) https://github.com/MontiCore/monticore */
//created by TestsMainEntry.ftl
#ifndef TESTS_MAIN
#define TESTS_MAIN
#include <iostream>
#include <stdio.h>

#include "test_math_floatDivisionTest_test.hpp"

int main(){
    std::cout << "=================Start stream testing=================" << std::endl;
    remove("stacktrace.log");
    int errorCode = 0;
    errorCode += test_math_floatDivisionTest_test::runTest();
    std::cout << "==================End stream testing==================" << std::endl;
    exit(errorCode);
}

#endif
