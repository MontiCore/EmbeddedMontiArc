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

// Dynamic Interface
EXPORT int DI__get_port_count();
EXPORT bool DI__is_output(int i);
EXPORT const char *DI__get_name(int i);
EXPORT const char *DI__get_type(int i);
EXPORT bool DI__allows_multiple_inputs(int i);
EXPORT bool DI__is_optional(int i);

// Simplified:
// Returns a JSON string deserializable to Object[] (of )
EXPORT const char* DI__get_interface();
EXPORT void DI__set_port(int i, const char* data);
EXPORT const char* DI__get_port(int i);

// Methods
EXPORT void init();
EXPORT void execute();

EXPORT void set__true_velocity(double v);
EXPORT void set__true_position(double v1, double v2);
EXPORT void set__true_compass(double v);
EXPORT void set_size__trajectory_x(int size);
EXPORT void set__trajectory_x(double v, int* i);
EXPORT void set_size__trajectory_y(int size);
EXPORT void set__trajectory_y(double v, int* i);
EXPORT double get__set_steering();
EXPORT double get__set_gas();
EXPORT double get__set_braking();

}
