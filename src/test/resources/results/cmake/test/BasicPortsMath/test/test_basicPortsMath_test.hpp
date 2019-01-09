#ifndef TEST_BASICPORTSMATH_TEST
#define TEST_BASICPORTSMATH_TEST

#include "catch.hpp"
#include "../test_basicPortsMath.h"
#include <iostream>
#include <fstream>
#include <string>
/*void toFileString(std::ofstream& myfile, mat A){
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
bool Is_close(mat& X, mat& Y, double tol)
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
void rangeValueCheck(double A, double lower, double upper){
    REQUIRE( A >= lower );
    REQUIRE( A <= upper );
}

void rangeValueCheck(int A, double lower, double upper){
    REQUIRE( A >= lower );
    REQUIRE( A <= upper );
}
void rangeValueCheck(mat& A, mat& lower , mat& upper){
    REQUIRE(Is_close(A, lower, 0.0001));
    REQUIRE(Is_close(A, upper, 0.0001));
}*/
TEST_CASE("test.BasicPortsMath", "[test_basicPortsMath]") {
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


#endif

