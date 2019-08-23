/* (c) https://github.com/MontiCore/monticore */
#include "config.h"

using namespace std;


bool str_equal( const char *first, uint size, const char *second ) {
    for ( uint i : urange( size ) )
        if ( second[i] == '\0' || second[i] != first[i] )
            return false;
    return second[size] == '\0';
}

MessageParser::MessageParser( const char *msg ) {
    this->msg = msg;
    next = msg;
}

bool MessageParser::has_next() {
    //Go to next line with content
    while ( *next == ' ' || *next == '\t' || *next == '\n' )
        ++next;
    //Check if at end of message
    if ( *next == 0 )
        return false;
        
    line_start = next;
    //Search the command name
    while ( *next != 0 && *next != '=' && *next != '\n' && *next != ' ' && *next != '\t' )
        ++next;
    if ( line_start == next )
        return false; //no command before '='
    cmd_size = ( uint )( next - line_start );
    //line_start[0;cmd_size] is now the command name
    
    while ( *next == ' ' || *next == '\t' ) //Skip optional whitespace
        ++next;
    //Check if the line has parameters
    if ( *next == '=' )
        this->rest = next + 1;
    else
        this->rest = nullptr;
        
    //Get the end of line
    while ( *next != 0 && *next != '\n' )
        ++next;
    return true;
}

bool MessageParser::is_cmd( const char *cmd ) {
    return str_equal( line_start, cmd_size, cmd );
}

bool MessageParser::cmd_starts_with( const char *cmd ) {
    for ( uint i : urange( cmd_size ) ) {
        if ( cmd[i] == '\0' )
            return true;
        if ( cmd[i] != line_start[i] )
            return false;
    }
    return cmd[cmd_size] == '\0';
}

bool MessageParser::get_long( slong &target ) {
    if ( rest == nullptr )
        return false;
    to_non_ws();
    if ( *rest == '\0' || *rest == '\n' )
        return false;
    const char *new_ptr;
    ulong val = strtoll( rest, ( char ** )&new_ptr, 10 );
    if ( new_ptr == rest )
        return false;
    target = val;
    rest = new_ptr;
    to_comma();
    if ( *rest != '\0' )
        ++rest;
    return true;
}

std::string MessageParser::get_string() {
    if ( rest == nullptr )
        return std::string();
    to_non_ws();
    if ( *rest == '\0' || *rest == '\n' )
        return std::string();
    auto str_start = rest;
    to_comma();
    if ( rest == str_start )
        return std::string();
    std::string res( str_start, rest - str_start );
    if ( *rest == ',' )
        ++rest;
    return res;
}

std::string MessageParser::get_cmd() {
    return std::string( line_start, cmd_size );
}

void MessageParser::to_non_ws() {
    while ( *rest == ' ' || *rest == '\t' )
        ++rest;
}

void MessageParser::to_comma() {
    while ( *rest != '\0' && *rest != '\n' && *rest != ',' )
        ++rest;
}

void MessageParser::unknown() {
    Log::err << Log::tag << "Unknown command: " << std::string( line_start ).substr( 0, cmd_size ) << "\n";
}

void MessageBuilder::add( const std::string &cmd, const std::string &param ) {
    if ( res.size() > 0 )
        res += '\n';
    res += cmd;
    if ( param.size() > 0 )
        res += '=' + param;
}
