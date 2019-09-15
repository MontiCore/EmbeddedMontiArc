/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
#include "hardware_emulator.h"
using namespace std;

VALUE_TYPE FunctionValue::get_type( const char *type_name ) {
    if ( strcmp( type_name, "Q" ) == 0 )
        return VALUE_TYPE::DOUBLE;
    else if ( strcmp( type_name, "Z" ) == 0 )
        return VALUE_TYPE::INT;
    else if ( strcmp( type_name, "CommonMatrixType" ) == 0 )
        return VALUE_TYPE::DOUBLE_ARRAY;
    else {
        Log::err << Log::tag << "Unsupported EMA Port type: " << type_name <<
                 ". \nAdd type in \"FunctionValue\", \"VALUE_TYPE\" (emulator/function_value.h/.cpp) and \"HardwareEmulator\"\n";
        return VALUE_TYPE::NONE;
    }
}
