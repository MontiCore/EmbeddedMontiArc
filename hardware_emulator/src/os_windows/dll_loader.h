#pragma once
#include <string>
#include "utility.h"
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "computer/computer.h"
#include "computer/os.h"
#include "computer/symbols.h"

namespace OS {

    struct DLLInfo {
        ulong base_address;
        ulong image_size;
        ulong base_of_code;
        ulong entry_point;
        ulong section_align;
        uint size_of_headers;
        
        void load_values( void *pe );
    };
    
    
    struct DLLLoader {
        void *pe;
        Array<char> file;
        std::string file_name;
        std::string module_name;
        bool module_name_set;
        
        SystemCalls *sys_calls;
        Symbols *symbols;
        Memory *mem;
        
        DLLInfo info;
        
        Array<SectionInfo> sections;
        uint section_pos;
        
        DLLLoader() : pe( nullptr ), sys_calls( nullptr ), mem( nullptr ), symbols( nullptr ) {}
        ~DLLLoader() {
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