<#-- (c) https://github.com/MontiCore/monticore -->
<#include "/Common.ftl">
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

EXPORT int get_input_count();
EXPORT const char *get_input_name(int id);
EXPORT const char *get_input_type(int id);

EXPORT int get_output_count();
EXPORT const char *get_output_name(int id);
EXPORT const char *get_output_type(int id);

EXPORT void init();
EXPORT void execute();

${viewModel.functionDeclarations}

}
