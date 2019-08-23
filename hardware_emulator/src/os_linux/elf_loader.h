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
#include <string>
#include "utility.h"
#include "computer/computer.h"
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "elf.h"

namespace OS {

    /*
        The ELFLoader can load a Linux shared object into the computer memory and resolve its
        symbols such as internal objects and system functions.
        It also registers the function of the program in the Symbol table.
    */
    struct ElfLoader {
        bool loaded;
        ElfFile elf;
        std::string file_name;
        std::string module_name;
        
        SystemCalls *sys_calls;
        Symbols *symbols;
        Memory *mem;
        
        Array<SectionInfo> sections;
        uint section_pos;
        
        ElfLoader() : sys_calls( nullptr ), mem( nullptr ), loaded( false ), symbols( nullptr ) {}
        
        bool init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem, Symbols &symbols );
        
        void elf_main( Computer &computer );
        
        
    };
    
    
}