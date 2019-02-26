//created by TestsMainEntry.ftl
<#include "/Common.ftl">
#ifndef TESTS_MAIN
#define TESTS_MAIN
#include <iostream>

<#list viewModel.includes as i>
    #include "${i}"
</#list>

int main(){
    std::cout << "=================Start stream testing=================" << std::endl;
    int errorCode = 0;
<#list viewModel.includes as i>
    errorCode += ${i?replace(".hpp","")}::runTest();
</#list>
    std::cout << "==================End stream testing==================" << std::endl;
    exit(errorCode);
}

#endif
