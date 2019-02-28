#pragma once
#include <string>
#include "utility.h"
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "computer/computer.h"
#include "elf.h"

namespace OS {

    struct ElfInfo {
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
    
    struct ElfLoader {
        bool loaded;
        ElfFile elf;
        std::string file_name;
        std::string module_name;
        bool module_name_set;
        
        SystemCalls *sys_calls;
        Memory *mem;
        
        ElfInfo info;
        
        Array<SectionInfo> sections;
        uint section_pos;
        
        ElfLoader() : sys_calls( nullptr ), mem( nullptr ), loaded(false) {}
        ~ElfLoader() {
            drop();
        }
        
        bool init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem );
        void drop();
        
        void dll_main( Computer &computer );
        
        
    };


}