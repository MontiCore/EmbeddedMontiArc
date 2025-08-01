/* (c) https://github.com/MontiCore/monticore */
//created by ComponentStreamTest2.ftl
#ifndef TEST_BASICPORTSMATH_TEST
#define TEST_BASICPORTSMATH_TEST

//#include "catch.hpp"
#include "../test_basicPortsMath.h"
#include <iostream>
#include <fstream>
#include <string>

namespace test_basicPortsMath_test{

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
    std::cout << "Actual value: " << A << std::endl;
    std::cout << "Lower bound for expected value: " << lower << std::endl;
    std::cout << "Upper bound for expected value: " << lower << std::endl;
    require_lower_or_equals(lower, A);
    require_lower_or_equals(A, upper);
}

void rangeValueCheck(int A, double lower, double upper){
    std::cout << "Actual value: " << A << std::endl;
    std::cout << "Lower bound for expected value: " << lower << std::endl;
    std::cout << "Upper bound for expected value: " << lower << std::endl;
    require_lower_or_equals(lower, A);
    require_lower_or_equals(A, upper);
}
void rangeValueCheck(mat& A, mat& lower , mat& upper){
    require_is_close(A, lower, 0.0001);
    require_is_close(A, upper, 0.0001);
}

void test_case_test_BasicPortsMath(){
    mat tmpA;
    mat tmpB;
    test_basicPortsMath component;
    std::ofstream myfile;
    myfile.open ("test.BasicPortsMath");
    myfile.precision(64);
    component.init();
            component.counter = -10.0;
        component.execute();

    rangeValueCheck(component.result, 0.0 , 0.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
            component.counter = -1.0;
        component.execute();

    rangeValueCheck(component.result, 0.0 , 0.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
            component.counter = 0.0;
        component.execute();

    rangeValueCheck(component.result, 0.0 , 0.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
            component.counter = 1.0;
        component.execute();

    rangeValueCheck(component.result, 1.0 , 1.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
            component.counter = 100.0;
        component.execute();

    rangeValueCheck(component.result, 100.0 , 100.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
            component.counter = 1000.0;
        component.execute();

    rangeValueCheck(component.result, 100.0 , 100.0);
            myfile << "result: ";
            toFileString(myfile, component.result);
            myfile << "\n";
    myfile.close();
    std::cout << "test.BasicPortsMath: success\n";
}

int runTest(){
    std::ofstream __StacktraceFile;
    assertions = 0;
    failedAssertions = 0;
    __StacktraceFile.open("stacktrace.log", std::ios_base::out | std::ios_base::app);
    __StacktraceFile << "~Entering test.BasicPortsMath" << std::endl;
    __StacktraceFile.close();
    test_case_test_BasicPortsMath();

    overallAssertions += assertions;
    overallFailedAssertions += failedAssertions;
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

