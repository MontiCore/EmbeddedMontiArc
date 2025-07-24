/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <string>
#include "utility/utility.h"
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
        uint64_t init_address = 0;
        Elf64_Dyn *dt_init_array_entry = nullptr;
        Elf64_Dyn *dt_init_array_size_entry = nullptr;
        
		std::vector<SectionInfo> sections;
        uint section_pos;
        
        ElfLoader() : sys_calls( nullptr ), mem( nullptr ), loaded( false ), symbols( nullptr ), section_pos(0) {}
        
        void init(const fs::path& file, SystemCalls &sys_calls, Memory &mem, Symbols &symbols );
        
        void elf_main( Computer &computer );
        
        
    };
    
    
}
