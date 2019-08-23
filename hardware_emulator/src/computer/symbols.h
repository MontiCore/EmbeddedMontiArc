/* (c) https://github.com/MontiCore/monticore */
#pragma once
#include <unordered_map>
#include "utility.h"

/*
    Wraps a simple name to symbol table using a hashmap.
*/
struct Symbols {
    struct Symbol {
        enum Type {
            NONE,
            SYSCALL,
            EXPORT, //Func from dll/lib
            OBJECT,
            HANDLE
        } type;
        ulong addr;
        
        Symbol() : type( NONE ), addr( 0 ) {}
        Symbol( Type type, ulong addr ) : type( type ), addr( addr ) {}
    };
    
    using SymbolTable = std::unordered_map<std::string, Symbol>;
    SymbolTable table;
    
    //Returns a NONE symbol if no match is found.
    Symbol get_symbol( const std::string &name );
    void add_symbol( const std::string &name, Symbol::Type type, ulong addr );
};
