#pragma once

/*
    This header should be used as an interface in an autopilot in order to:
    - throw errors
    - print error messages (cerr)
    - print information messages (cout)

    The correct function pointers must be provided to the autopilot through the ERR_OUT_set_functions() function

    The goal is that the hardware_emulator can substitute its own function calls (dynamically loaded)
    in order to properly transmit messages and errors from the autopilot.

*/

#ifdef __cplusplus
extern "C" {
#endif

using throw_func = void (*)(const char* type, const char* message);
using print_func = void (*)(const char* message);
using set_functions_func = void (*)(throw_func throw_error_ptr, print_func print_cout_ptr, print_func print_cerr_ptr);

extern throw_func ERR_OUT_throw_error;
extern print_func ERR_OUT_print_cout;
extern print_func ERR_OUT_print_cerr;

void ERR_OUT_set_functions(throw_func throw_error_ptr, print_func print_cout_ptr, print_func print_cerr_ptr);

#ifndef NO_ERR_OUT
void throw_error(const char *type, const char *msg_format, ...);
void print_cout(const char *msg_format, ...);
void print_cerr(const char *msg_format, ...);
#endif

#ifdef __cplusplus
}
#endif
