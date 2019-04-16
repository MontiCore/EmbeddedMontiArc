//created by ComponentStreamTest2.ftl
<#include "/Common.ftl">
#ifndef ${viewModel.fileNameWithoutExtension?upper_case}
#define ${viewModel.fileNameWithoutExtension?upper_case}

//#include "catch.hpp"
#include "../${viewModel.componentName}.h"
#include <iostream>
#include <fstream>
#include <string>

namespace ${viewModel.fileNameWithoutExtension}{

void toFileString(std::ofstream& myfile, mat A){
    myfile << "[";
    for (int i = 0; i < A.n_rows; i++){
        for (int j = 0; j < A.n_cols; j++){
            myfile << A(i,j);
            if(j + 1 < A.n_cols){
                myfile << ", ";
            }
        }
        if(i + 1 < A.n_rows){
            myfile << ";";
        }
    }
    myfile << "]";
}
void toFileString(std::ofstream& myfile, double A){
    myfile << A;
}
void toFileString(std::ofstream& myfile, float A){
    myfile << A;
}
void toFileString(std::ofstream& myfile, int A){
    myfile << A;
}
void toFileString(std::ofstream& myfile, bool A){
    myfile << A;
}

bool isClose(mat& X, mat& Y, double tol)
{
    // abs returns a mat type then max checks columns and returns a row_vec
    // max used again will return the biggest element in the row_vec
    bool close(false);
    if(arma::max(arma::max(arma::abs(X-Y))) < tol)
    {
        close = true;
    }
    return close;
}

int overallAssertions = 0;
int overallFailedAssertions = 0;
int assertions = 0;
int failedAssertions = 0;

void require_is_close(mat& A, mat& lower, double tol){
    assertions++;
    if(!isClose(A, lower, tol)){
        std::cout << "Failed at: isClose(" << A << ", " << lower << ", " << tol << std::endl;
        failedAssertions++;
    }
}

void require_boolean(bool a){
    assertions++;
    if(!a){
        std::cout << "Failed at: " << a << " != true" << std::endl;
        failedAssertions++;
    }
}

void require_boolean_false(bool a){
    assertions++;
    if(a){
        std::cout << "Failed at: " << a << " != false" << std::endl;
        failedAssertions++;
    }
}

void require_lower_or_equals(double a, double b){
    assertions++;
    if(a > b){
        std::cout << "Failed at: " << a << " <= " << b << std::endl;
        failedAssertions++;
    }
}

void rangeValueCheck(double A, double lower, double upper){
    require_lower_or_equals(lower, A);
    require_lower_or_equals(A, upper);
}

void rangeValueCheck(int A, double lower, double upper){
    require_lower_or_equals(lower, A);
    require_lower_or_equals(A, upper);
}
void rangeValueCheck(mat& A, mat& lower , mat& upper){
    require_is_close(A, lower, 0.0001);
    require_is_close(A, upper, 0.0001);
}

<#list viewModel.streams as stream>
void test_case_${stream.name?replace(".","_")}(){
    mat tmpA;
    mat tmpB;
    ${viewModel.componentName} component;
    std::ofstream myfile;
    myfile.open ("${stream.name}");
    myfile.precision(64);
    component.init();
    <#list stream.checks as check>
        <#list check.inputPortName2Value?keys as portName>
            component.${portName} ${check.inputPortName2Value[portName]};
        </#list>
        component.execute();
        <#list check.outputPortName2Check?keys as outputPortName>
            <@renderPortCheck outputPortName=outputPortName check=check.outputPortName2Check[outputPortName] />
        </#list>
        <#list stream.outputPortNames as outputPortName>
            myfile << "${outputPortName}: ";
            toFileString(myfile, component.${outputPortName});
            myfile << "\n";
        </#list>
    </#list>
    myfile.close();
    std::cout << "${stream.name}: success\n";
}
</#list>

int runTest(){
    std::ofstream __StacktraceFile;
<#list viewModel.streams as stream>
    assertions = 0;
    failedAssertions = 0;
    __StacktraceFile.open("stacktrace.log", std::ios_base::out | std::ios_base::app);
    __StacktraceFile << "~Entering ${stream.name}" << std::endl;
    __StacktraceFile.close();
    test_case_${stream.name?replace(".","_")}();

    overallAssertions += assertions;
    overallFailedAssertions += failedAssertions;
</#list>
    if(overallFailedAssertions == 0){
        std::cout << "All tests passed! Made " << overallAssertions << " assertions." << std::endl;
        return 0;
    }else{
        std::cout << "There are failed tests! Failed " << overallFailedAssertions << " of " << overallAssertions << " assertions." << std::endl;
        return 1;
    }
}
}
#endif

<#macro renderPortCheck outputPortName check>
<#assign portValue="component.${outputPortName}">
<#if helper.isBooleanOutputPortCheck(check)>
    <#if helper.isTrueExpectedCheck(check)>
        require_boolean( ${portValue} );
    <#else>
        require_boolean_false( ${portValue} );
    </#if>
<#elseif helper.isRangeOutputPortCheck(check)>

    <#if check.isMatrix??>
    <#if check.isMatrix()>
    tmpA = mat();
    tmpA  ${check.lowerBound};
    tmpB = mat();
    tmpB  ${check.upperBound};
    rangeValueCheck(${portValue}, tmpA , tmpB);
    <#else>
    rangeValueCheck(${portValue}, ${check.lowerBound} , ${check.upperBound});
    </#if>
    <#else>
    rangeValueCheck(${portValue}, ${check.lowerBound} , ${check.upperBound});
    </#if>
<#else>

</#if>
</#macro>
