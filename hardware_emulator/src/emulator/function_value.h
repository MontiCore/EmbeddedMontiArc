/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
		std::vector<double> data;
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
            double_array.data = std::vector<double>( data, data+size );
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
