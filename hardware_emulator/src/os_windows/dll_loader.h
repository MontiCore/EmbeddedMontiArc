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
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "computer/computer.h"
#include "computer/os.h"
#include "computer/symbols.h"

namespace OS {



    /*
        This component is responsible for loading a DLL file into the computer memory
        and expose the symbols (functions) of the file.
    
        dll_main() calls the entry point of DLLs.
    */
    struct DllLoader {
        void *pe;
        Array<char> file;
        std::string file_name;
        std::string module_name;
        bool module_name_set;
        
        SystemCalls *sys_calls;
        Symbols *symbols;
        Memory *mem;
        
        struct DLLInfo {
            ulong base_address;
            ulong image_size;
            ulong base_of_code;
            ulong entry_point;
            ulong section_align;
            uint size_of_headers;
            
            void load_values( void *pe );
        } info;
        
        
        
        Array<SectionInfo> sections;
        uint section_pos;
        
        DllLoader() : pe( nullptr ), sys_calls( nullptr ), mem( nullptr ), symbols( nullptr ) {}
        ~DllLoader() {
            drop();
        }
        
        //File without extension
        bool init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem, Symbols &symbols );
        void drop();
        bool loaded() {
            return pe != nullptr;
        }
        
        void dll_main( Computer &computer );
        
        
    };
    
}