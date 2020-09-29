#pragma once
#include <stdint.h>
#if defined(_MSC_VER)
    //  Microsoft 
    #define EXPORT __declspec(dllexport)
    #define IMPORT __declspec(dllimport)
#elif defined(__GNUC__)
    //  GCC
    #define EXPORT __attribute__((visibility("default")))
    #define IMPORT
#else
    //  do nothing and hope for the best?
    #define EXPORT
    #define IMPORT
    #pragma warning Unknown dynamic link import/export semantics.
#endif

extern "C" {
    
    
    EXPORT int32_t int_one( int32_t );
    EXPORT int32_t int_two( int32_t,int32_t );
    EXPORT int32_t int_three( int32_t,int32_t,int32_t );
    EXPORT int32_t int_four( int32_t,int32_t,int32_t,int32_t );
    
    
    EXPORT int64_t long_one( int64_t);
    EXPORT int64_t long_two( int64_t,int64_t );
    EXPORT int64_t long_three( int64_t,int64_t,int64_t);
    EXPORT int64_t long_four( int64_t,int64_t,int64_t,int64_t);
    
    
    EXPORT float float_one( float);
    EXPORT float float_two( float,float );
    EXPORT float float_three( float,float,float);
    EXPORT float float_four( float,float,float,float);
    
    
    EXPORT double double_one( double);
    EXPORT double double_two( double,double );
    EXPORT double double_three( double,double,double);
    EXPORT double double_four( double,double,double,double);
    
    EXPORT double double_array(double*, int);
    EXPORT float float_array(float*, int);
    EXPORT int32_t int_array(int32_t*, int);
    EXPORT int64_t long_array(int64_t*, int);
    EXPORT char char_array(char*, int);
}
