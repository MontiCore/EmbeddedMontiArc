#include "err_out.h"
#include "buffer.h"
#include <stdarg.h>
#include "printf.h"


throw_func ERR_OUT_throw_error = nullptr;
print_func ERR_OUT_print_cout = nullptr;
print_func ERR_OUT_print_cerr = nullptr;


void ERR_OUT_set_functions(throw_func throw_error_ptr, print_func print_cout_ptr, print_func print_cerr_ptr) {
    ERR_OUT_throw_error = throw_error_ptr;
    ERR_OUT_print_cout = print_cout_ptr;
    ERR_OUT_print_cerr = print_cerr_ptr;
}

void throw_error(const char *type, const char *msg_format, ...) {
    va_list args;
    va_start(args, msg_format);
    vsnprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, msg_format, args);
    va_end(args);
    ERR_OUT_throw_error(type, LOCAL_BUFFER);
}
void print_cout(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, format, args);
    va_end(args);
    ERR_OUT_print_cout(LOCAL_BUFFER);
}
void print_cerr(const char *format, ...) {
    va_list args;
    va_start(args, format);
    vsnprintf(LOCAL_BUFFER, LOCAL_BUFFER_SIZE, format, args);
    va_end(args);
    ERR_OUT_print_cerr(LOCAL_BUFFER);
}