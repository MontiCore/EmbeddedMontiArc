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