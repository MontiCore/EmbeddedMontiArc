/* (c) https://github.com/MontiCore/monticore */
#pragma once

#include "utility.h"


/*
    In-memory parser of config and query messages.
    Iterate through the lines of the message using has_next()
    Use is_cmd() to compare the command name or get_cmd() to get the command name directly.

    get_string() will return the next parameter as string, trimming of any whitespace.
    get_long() will try to return the next parameter as long inside the target variable.
    These functions advance to the next parameter.

    Ex:
    MessageParser msg("value=3\nname=jakck, black");
    while(msg.has_next()){
        if (msg.is_cmd("value")){
            slong value;
            if (!msg.get_long(value)){
                Log::err << "Expected long after 'value'\n";
            } else {
                ...
            }
        } else if (msg.is_cmd("name")){
            auto first_name = msg.get_string();
            auto last_name = msg.get_string();
            ...
        } else
            Log::err << "Unknown command\n";
    }
*/
struct MessageParser {
    const char *msg;
    uint cmd_size;
    const char *rest;
    const char *line_start;
    const char *next;
    
    MessageParser( const char *msg );
    bool has_next();
    bool is_cmd( const char *cmd );
    bool cmd_starts_with( const char *cmd );
    bool get_long( slong &target );
    std::string get_string();
    
    std::string get_cmd();
    
    void to_non_ws();
    void to_comma();
    
    void unknown();
};

/*
    Simple builder of config and query messages.
    add() adds a new line to the message
    multiple arguments have to be manually separated by commas.
*/
struct MessageBuilder {
    std::string res;
    void add( const std::string &cmd, const std::string &param = std::string() );
};
