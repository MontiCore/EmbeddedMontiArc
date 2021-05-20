#pragma once

/*
    This header should be used as an abstraction in an autopilot in order to:
    - throw errors
    - print error messages (cerr)
    - print information messages (cout)

    The goal is that the hardware_emulator can substitute its own function calls (dynamically loaded)
    in order to properly transmit messages and errors from the autopilot.

    In order to use the different implementations (standalone_err_out.cpp, ...) the correct file has to be compiled into the autopilot.
*/

#ifdef __cplusplus
extern "C" {
#endif


void throw_error(const char* type, const char* message);
void print_cout(const char *message);
void print_cerr(const char *message);


#ifdef __cplusplus
}
#endif
