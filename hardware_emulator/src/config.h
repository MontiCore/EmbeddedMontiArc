#pragma once

#include "utility.h"

struct MessageParser {
    const char *msg;
    uint cmd_size;
    const char *rest;
    const char *line_start;
    const char *next;
    
    MessageParser( const char *msg );
    bool has_next();
    bool is_cmd( const char *cmd );
    bool get_long( slong &target );
    std::string get_string();
    
    void to_non_ws();
    void to_comma();
    
    void unknown();
};

struct MessageBuilder {
    std::string res;
    void add( const std::string &cmd, const std::string &param = std::string() );
};