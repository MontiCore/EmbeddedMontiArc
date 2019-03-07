#include "symbols.h"

Symbols::Symbol Symbols::get_symbol( const std::string &name ) {
    auto s_res = table.find( name );
    if ( s_res == table.end() )
        return Symbol();
    else
        return s_res->second;
}

void Symbols::add_symbol( const std::string &name, Symbol::Type type, ulong addr ) {
    table.insert( std::pair( name, Symbol( type, addr ) ) );
}

void Symbols::add_symbol( const std::string &name, Symbol::Type type, ulong addr, ulong param1 ) {
    table.insert( std::pair( name, Symbol( type, addr, param1 ) ) );
}
