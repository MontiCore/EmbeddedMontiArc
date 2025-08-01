#include "standard_err_out.h"
#include <exception>
#include <string>
#include <iostream>

class StandaloneException : public std::exception {
    std::string msg;
public:
    StandaloneException(const char* type, const char* message) : msg((std::string(type) + ": ") + message) {}
    virtual const char* what() const throw()
    {
        return msg.c_str();
    }
};

void ERR_OUT_standard_throw_error(const char* type, const char* message) {
    throw StandaloneException(type, message);
}
void ERR_OUT_standard_print_cout(const char *message) {
    std::cout << message << std::endl;
}
void ERR_OUT_standard_print_cerr(const char *message) {
    std::cerr << message << std::endl;
}