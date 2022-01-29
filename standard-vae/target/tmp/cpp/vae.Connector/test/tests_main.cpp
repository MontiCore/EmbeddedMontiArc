//created by TestsMainEntry.ftl
#ifndef TESTS_MAIN
#define TESTS_MAIN
#include <iostream>
#include <stdio.h>


int main(){
    std::cout << "=================Start stream testing=================" << std::endl;
    remove("stacktrace.log");
    int errorCode = 0;
    std::cout << "==================End stream testing==================" << std::endl;
    exit(errorCode);
}

#endif
