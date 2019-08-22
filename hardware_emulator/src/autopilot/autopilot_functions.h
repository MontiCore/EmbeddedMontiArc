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
#pragma once
#include <string>
#include "utility.h"
#include "autopilot_port_info.h"




struct AutopilotFunction {
    static const char *module_name;
    static AutopilotFunction autopilot_inputs[AUTOPILOT_INPUT_COUNT];
    static AutopilotFunction autopilot_outputs[AUTOPILOT_OUTPUT_COUNT];
    
    const char *name;
    VALUE_TYPE type;
    static std::string get_jni_name( const std::string &name ) {
        auto u_pos = name.find( '_' );
        if ( u_pos == std::string::npos )
            return name;
        std::string res;
        size_t last_pos = 0;
        do {
            ++u_pos;
            res += name.substr( last_pos, u_pos - last_pos );
            res += "1";
            last_pos = u_pos;
            u_pos = name.find( '_', u_pos );
        } while ( u_pos != std::string::npos );
        res += name.substr( last_pos );
        return res;
    }
    std::string get_input_name() {
        return std::string( module_name ) + "set_1" + get_jni_name( name );
    }
    std::string get_output_name() {
        return std::string( module_name ) + "get_1" + get_jni_name( name );
    }
    static std::string get_name( const char *name ) {
        return std::string( module_name ) + get_jni_name( name );
    }
};
