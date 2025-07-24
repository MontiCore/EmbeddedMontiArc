/** (c) https://github.com/MontiCore/monticore */
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

// See https://git.rwth-aachen.de/monticore/EmbeddedMontiArc/simulators/simulation/-/wikis/dev-docs/concepts/Library-Interface

EXPORT const char* DI__get_interface();

EXPORT void DI__set_port(int i, const char* data, int is_json);
// For "Socket" type ports: returns data as long as there is data in the queue
// When empty: returns the empty string ("") or an empty binary payload (size 0)
EXPORT const char* DI__get_port(int i, int is_json);

EXPORT void DI__init();
EXPORT void DI__execute(double delta_sec);

}
