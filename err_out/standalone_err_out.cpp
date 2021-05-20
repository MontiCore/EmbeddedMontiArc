#include "err_out.h"
#include <exception>
#include <string>
#include <iostream>

/*
    This "backend" for err_out.h just uses the standard outputs and throws a standard c++ exception.
*/

class StandaloneException : public std::exception {
    std::string msg;
public:
    StandaloneException(const char* type, const char* message) : msg((std::string(type) + ": ") + message) {}
    virtual const char* what() const throw()
    {
        return msg.c_str();
    }
};

void throw_error(const char* type, const char* message) {
    throw StandaloneException(type, message);
}
void print_cout(const char *message) {
    std::cout << message;
}
void print_cerr(const char *message) {
    std::cerr << message;
}