#include "os_linux.h"


void OS::Linux::init( Computer &computer ) {
}

ulong OS::Linux::add_symbol( const std::string &mod, const std::string &name, uint size, Annotation::Type type ) {
    return ulong();
}

bool OS::Linux::load_file( const char *file ) {
    return false;
}
