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

// Returns a JSON string deserializable to the ProgramInterface class (Commons).
EXPORT const char* DI__get_interface();
EXPORT void DI__set_port(int i, const char* data);
EXPORT const char* DI__get_port(int i);

// Methods
EXPORT void DI__init();
EXPORT void DI__execute(double delta_sec);

}
