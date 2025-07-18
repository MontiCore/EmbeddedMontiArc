#pragma once

/*
    A set of functions that can be given to the ERR_OUT_set_functions() call of an autopilot.
    These functions redirect the calls to cout and cerr and throw a C++ exception.
*/

#ifdef __cplusplus
extern "C" {
#endif

void ERR_OUT_standard_throw_error(const char* type, const char* message);
void ERR_OUT_standard_print_cout(const char *message);
void ERR_OUT_standard_print_cerr(const char *message);

#ifdef __cplusplus
}
#endif
