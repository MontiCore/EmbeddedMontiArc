<#-- (c) https://github.com/MontiCore/monticore -->
//created by ComponentStreamTest2.ftl
<#include "/Common.ftl">
#ifndef ${viewModel.fileNameWithoutExtension?upper_case}
#define ${viewModel.fileNameWithoutExtension?upper_case}

//#include "catch.hpp"
#include "../${viewModel.componentName}.h"
#include <iostream>
#include <fstream>
#include <string>
<#if viewModel.isUseOpenCV()>
#include "CNNTranslator.h"
#include "ImageMatcher.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
</#if>

namespace ${viewModel.fileNameWithoutExtension}{

void toFileString(std::ofstream& myfile, icube A){
    cube tmp = conv_to<cube>::from(A);
    myfile << "[";
    for (int k = 0; k < tmp.n_slices; ++k) {
        myfile << "[";
        for (int i = 0; i < tmp.n_rows; i++){
            for (int j = 0; j < tmp.n_cols; j++){
                myfile << tmp(i,j,k);
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

    myfile << "]";
}

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

bool isClose(cube& X, cube& Y, double eleTol, double genTol)
{
    cube tmp = arma::abs(X - Y);
    int maxNumberOfError = floor(X.n_elem * genTol);
    int numberOfError = 0;
    for (int i = 0; i < tmp.n_slices; ++i) {
        for (int j = 0; j < tmp.n_rows; ++j) {
            for (int k = 0; k < tmp.n_cols; ++k) {
                if (tmp(j, k, i) > eleTol) {
                    numberOfError++;
                }
            }
        }
    }
    return numberOfError <= maxNumberOfError;
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

void require_is_close(cube& A, cube& B, double eleTol, double genTol){
    assertions++;
    bool is_same_size = (arma::size(A) == arma::size(B));
    if(!is_same_size) {
        std::cout << "Failed at size: given test cube is of a different size:" << size(B) << std::endl;
    }
    if(!isClose(A, B, eleTol, genTol)){
        std::cout << "Failed" << std::endl;
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
void rangeValueCheck(cube& A, cube& B , double eleTol, double genTol){
    require_is_close(A, B, eleTol, genTol);
}

<#list viewModel.streams as stream>
void test_case_${stream.name?replace(".","_")}(){
    mat tmpA;
    mat tmpB;
    cube tmpC;
    cube tmpD;
    <#if viewModel.isUseOpenCV()>
    vector<float> img_vec;
    cv::Mat img;
    cv::Mat result_img_cv;
    cv::Mat test_img_cv;
    </#if>

    ${viewModel.componentName} component;
    std::ofstream myfile;
    myfile.open ("${stream.name}");
    myfile.precision(64);
    component.init();
    <#list stream.checks as check>
        <#list check.inputPortName2Value?keys as portName>
            <@inputPort inputPortName=portName input=check.inputPortName2Value[portName] />
        </#list>
        component.execute();
        <#list check.outputPortName2Check?keys as outputPortName>
            <@renderPortCheck outputPortName=outputPortName check=check.outputPortName2Check[outputPortName] number=check?index />
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

<#macro inputPort inputPortName input>
    <#if input.isImage()>
        cv::samples::addSamplesDataSearchPath("${input.getWorkingDirectory()}");
        img = imread( samples::findFile( "${input.getImagePath()}" ));
        component.${inputPortName} = CNNTranslator::translateImageToICube(img, ${input.getN_slices()}, ${input.getN_rows()}, ${input.getN_cols()});
    <#else>
    <#if input.getIsCube()>
        tmpC = cube(${input.getN_rows()}, ${input.getN_cols()}, ${input.getN_slices()});
        <#list input.inputList as matrix>
            tmpA = mat();
            tmpA ${matrix};
            tmpC.slice(${matrix?index}) = tmpA;
        </#list>
        component.${inputPortName} = conv_to< icube >::from(tmpC);
    <#else>
        component.${inputPortName} ${input.input};
    </#if>
    </#if>
</#macro>

<#macro renderPortCheck outputPortName check number>
<#assign portValue="component.${outputPortName}">
<#if helper.isBooleanOutputPortCheck(check)>
    <#if helper.isTrueExpectedCheck(check)>
        require_boolean( ${portValue} );
    <#else>
        require_boolean_false( ${portValue} );
    </#if>
<#elseif helper.isRangeOutputPortCheck(check)>
    <#if check.isImage()>
        img_vec = CNNTranslator::translate(conv_to<cube>::from(${portValue}));
        result_img_cv = cv::Mat(img_vec, false);
        result_img_cv = result_img_cv.reshape(${check.getN_slices()}, ${check.getN_rows()});
        result_img_cv.convertTo(result_img_cv, CV_8UC1);
        cv::imwrite("${check.getImagePathWithoutExtension()}_generated_${number}.png", result_img_cv);
        tmpC = conv_to<cube>::from(CNNTranslator::translateImageToICube(result_img_cv, ${check.getN_slices()}, ${check.getN_rows()}, ${check.getN_cols()}));

        cv::samples::addSamplesDataSearchPath("${check.getWorkingDirectory()}");
        test_img_cv = imread( samples::findFile( "${check.getImagePath()}" ), 0 );
        test_img_cv = test_img_cv.reshape(${check.getN_slices()}, ${check.getN_rows()});
        test_img_cv.convertTo(test_img_cv, CV_8UC1);
        tmpD = conv_to<cube>::from(CNNTranslator::translateImageToICube(test_img_cv, ${check.getN_slices()}, ${check.getN_rows()}, ${check.getN_cols()}));
        std::cout << "Current test file is: " << "${check.getImagePath()}" << std::endl;
        rangeValueCheck(tmpD, tmpC , ${check.getElementTolerance()}, ${check.getGeneralTolerance()});
        ImageMatcher::drawMatch(result_img_cv, test_img_cv, "${check.getImagePathWithoutExtension()}_matcher_${number}.png");
    <#else>

    <#if check.isMatrix??>
    <#if check.isMatrix()>
        <#if check.isCube()>
            tmpC = cube(${check.getN_rows()}, ${check.getN_cols()}, ${check.getN_slices()});
            <#list check.lowerBoundList as matrix>
                tmpA = mat();
                tmpA ${matrix};
                tmpC.slice(${matrix?index}) = tmpA;
            </#list>
            tmpD = conv_to<cube>::from(${portValue});
            rangeValueCheck(tmpD, tmpC , ${check.getElementTolerance()}, ${check.getGeneralTolerance()});
        <#else>
            tmpA = mat();
            tmpA  ${check.lowerBound};
            tmpB = mat();
            tmpB  ${check.upperBound};
            rangeValueCheck(${portValue}, tmpA , tmpB);
        </#if>
    <#else>
    rangeValueCheck(${portValue}, ${check.lowerBound} , ${check.upperBound});
    </#if>
    <#else>
    rangeValueCheck(${portValue}, ${check.lowerBound} , ${check.upperBound});
    </#if>
    </#if>

<#else>

</#if>
</#macro>
