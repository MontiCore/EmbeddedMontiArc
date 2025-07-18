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

// These are the functions that any autopilot must implement in order to be loaded and executed by the hardware_emulator

// These are wrappers around the "autopilot interface functions" (program.h)
// These are simpler entry points so that the hardware_emulator can more easily communicate with the autopilot
// (Simple pure C types & simple function calling standards) 

// Returns The JSON string of the DynamicInterface
EXPORT const char* DI__get_interface();

// Port data-exchange functions
// If is_json != 0, the data is a JSON string
// Else the data is binary: First 4 bytes of data -> size of the rest
// Same for the returned data from DI__get_port()
EXPORT void DI__set_port(int i, const char* data, int is_json);
// For "Socket" type ports: returns data as long as there is data in the queue
// When empty: returns the empty string ("") or an empty binary payload (size 0)
EXPORT const char* DI__get_port(int i, int is_json);

// Methods
EXPORT void DI__init();
EXPORT void DI__execute(double delta_sec);

}
