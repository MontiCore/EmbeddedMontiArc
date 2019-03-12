#pragma once
#include <unordered_map>
#include "utility.h"


struct Symbols {
    struct Symbol {
        enum Type {
            NONE,
            SYSCALL, //param1 is syscall_id
            EXPORT, //Func from dll/lib
            OBJECT,
            HANDLE
        } type;
        ulong addr;
        ulong param1;
        
        Symbol() : type( NONE ), addr( 0 ), param1( 0 ) {}
        Symbol( Type type, ulong addr ) : type( type ), addr( addr ), param1( 0 ) {}
        Symbol( Type type, ulong addr, ulong param1 ) : type( type ), addr( addr ), param1( param1 ) {}
    };
    
    using SymbolTable = std::unordered_map<std::string, Symbol>;
    SymbolTable table;
    
    Symbol get_symbol( const std::string &name );
    void add_symbol( const std::string &name, Symbol::Type type, ulong addr );
    void add_symbol( const std::string &name, Symbol::Type type, ulong addr, ulong param1 );
};