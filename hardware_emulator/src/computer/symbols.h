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