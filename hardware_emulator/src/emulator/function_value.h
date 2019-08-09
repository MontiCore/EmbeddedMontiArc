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
#include "utility.h"


enum class VALUE_TYPE {
    NONE,
    DOUBLE,
    INT,
    DOUBLE_ARRAY
};

/*
    This structure holds a data type and a buffer for the data type.
    New data types require update of this buffer structure.
*/
struct FunctionValue {
    VALUE_TYPE type;
    struct {
        uint size;
        Array<double> data;
    } double_array;
    double double_value;
    int int_value;
    
    
    void init( double double_value ) {
        this->double_value = double_value;
        this->type = VALUE_TYPE::DOUBLE;
    }
    void init( int int_value ) {
        this->int_value = int_value;
        this->type = VALUE_TYPE::INT;
    }
    void init( uint size, double *data ) {
        this->double_array.size = size;
        this->type = VALUE_TYPE::DOUBLE_ARRAY;
        if ( size != 0 )
            double_array.data.init( size, data );
    }
    
    void init( VALUE_TYPE type ) {
        this->type = type;
        switch ( type ) {
            case VALUE_TYPE::DOUBLE: double_value = 0; break;
            case VALUE_TYPE::INT: int_value = 0; break;
            case VALUE_TYPE::DOUBLE_ARRAY: double_array.size = 0; break;
        }
    }
    
    
    static VALUE_TYPE get_type( const char *type_name );
};
