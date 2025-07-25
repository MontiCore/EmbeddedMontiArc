/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once


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
EXPORT int lib_bar_function();
}



struct Foo {
    int a;

    Foo();
};

namespace foo_ns {
    extern Foo global_foo;
}