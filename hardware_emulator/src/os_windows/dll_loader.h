#pragma once
#include <string>
#include "utility.h"
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "computer/computer.h"

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
    
    struct SectionInfo {
        MemorySection *mem;
    };
    
    struct DLLLoader {
        void *pe;
        Array<char> file;
        std::string file_name;
        
        SystemCalls *sys_calls;
        Memory *mem;
        
        DLLInfo info;
        
        Array<SectionInfo> sections;
        uint section_pos;
        
        DLLLoader() : pe( nullptr ), sys_calls( nullptr ), mem( nullptr ) {}
        ~DLLLoader() {
            drop();
        }
        
        bool init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem );
        void drop();
        bool loaded() {
            return pe != nullptr;
        }
        
        void dll_main( Computer &computer );
        
        
    };
    
    
    
    
}