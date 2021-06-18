#include "err_out.h"


throw_func throw_error = nullptr;
print_func print_cout = nullptr;
print_func print_cerr = nullptr;


void ERR_OUT_set_functions(throw_func throw_error_ptr, print_func print_cout_ptr, print_func print_cerr_ptr) {
    throw_error = throw_error_ptr;
    print_cout = print_cout_ptr;
    print_cerr = print_cerr_ptr;
}
