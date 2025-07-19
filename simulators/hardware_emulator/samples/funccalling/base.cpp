/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "base.h"
#include "algorithm.h"

EXPORT int32_t int_one( int32_t t1){
    return combine<int32_t>(t1);
}
EXPORT int32_t int_two( int32_t t1, int32_t t2 ){
    return combine<int32_t>(t1,t2);
}
EXPORT int32_t int_three( int32_t t1, int32_t t2, int32_t t3 ){
    return combine<int32_t>(t1,t2,t3);
}
EXPORT int32_t int_four( int32_t t1, int32_t t2, int32_t t3, int32_t t4){
    return combine<int32_t>(t1,t2,t3,t4);
}


EXPORT int64_t long_one( int64_t t1){
    return combine<int64_t>(t1);
}
EXPORT int64_t long_two( int64_t t1, int64_t t2 ){
    return combine<int64_t>(t1,t2);
}
EXPORT int64_t long_three( int64_t t1, int64_t t2, int64_t t3){
    return combine<int64_t>(t1,t2,t3);
}
EXPORT int64_t long_four( int64_t t1, int64_t t2, int64_t t3, int64_t t4){
    return combine<int64_t>(t1,t2,t3,t4);
}


EXPORT float float_one( float t1){
    return combine<float>(t1);
}
EXPORT float float_two( float t1,float t2 ){
    return combine<float>(t1,t2);
}
EXPORT float float_three( float t1,float t2,float t3){
    return combine<float>(t1,t2,t3);
}
EXPORT float float_four( float t1,float t2,float t3 ,float t4){
    return combine<float>(t1,t2,t3,t4);
}


EXPORT double double_one( double t1){
    return combine<double>(t1);
}
EXPORT double double_two( double t1,double t2 ){
    return combine<double>(t1,t2);
}
EXPORT double double_three( double t1,double t2,double t3){
    return combine<double>(t1,t2,t3);
}
EXPORT double double_four( double t1,double t2,double t3,double t4){
    return combine<double>(t1,t2,t3,t4);
}

EXPORT double double_array(double*a, int s){
    return combine_array<double>(a,s);
}
EXPORT float float_array(float*a, int s){
    return combine_array<float>(a,s);
}
EXPORT int32_t int_array(int32_t*a, int s){
    return combine_array<int32_t>(a,s);
}
EXPORT int64_t long_array(int64_t*a, int s){
    return combine_array<int64_t>(a,s);
}
EXPORT char char_array(char*a, int s){
    return combine_array<char>(a,s);
}

EXPORT int32_t test_long_function() {

}

EXPORT int32_t test_long_function_intern() {
    int a = 19 * 2;
    a += 4;
    auto res = MultiByteToWideChar_TEST(a, a+1, (const char*)(a+2), a+3, (uint16_t *)(a+4), a+5);
    res -= 321;
    return res;
}

EXPORT int32_t MultiByteToWideChar_TEST(
    uint32_t CodePage, uint32_t dwFlags, 
    const char* lpMultiByteStr, int32_t cbMultiByte,
    uint16_t *lpWideCharStr, int32_t cchWideChar
) {
    int32_t res = CodePage * 2;
    res += dwFlags * 3;
    res += (int32_t)lpMultiByteStr * 4;
    res += cbMultiByte * 5;
    res += (int32_t)lpWideCharStr * 6;
    res += cchWideChar * 7;
    return res;
}