#include "hardware_emulator.h"
using namespace std;

VALUE_TYPE FunctionValue::get_type( const char *type_name ) {
    if ( strcmp( type_name, "Q" ) == 0 )
        return VALUE_TYPE::DOUBLE;
    else if ( strcmp( type_name, "Z" ) == 0 )
        return VALUE_TYPE::INT;
    else if ( strcmp( type_name, "CommonMatrixType" ) == 0 )
        return VALUE_TYPE::DOUBLE_ARRAY;
    else
        return VALUE_TYPE::NONE;
}
