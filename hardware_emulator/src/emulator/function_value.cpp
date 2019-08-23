/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
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
