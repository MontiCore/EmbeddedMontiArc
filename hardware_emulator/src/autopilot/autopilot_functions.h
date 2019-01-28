#pragma once
#include <string>
#include "utility.h"
#include "autopilot_port_info.h"
enum class VALUE_TYPE {
    NONE,
    DOUBLE,
    INT,
    DOUBLE_ARRAY
};
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
};



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
