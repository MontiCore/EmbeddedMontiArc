/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "computer/os.h"
#include "computer/function_calling.h"
#include "elf_loader.h"
#include "linux_fast_call.h"

namespace OS {

    
    /*
        The Linux operating system implementation.
    
        It registers the LinuxFastCall implementation in the computer, registers the Linux system functions.
    
        load_file() loads a Linux program into the computer memory and exposes its functions.
    */
    struct Linux : public OS {
        MemorySection *section;
        SectionStack *section_stack;
        LinuxFastCall& func_call;
        
        
        ElfLoader elf;
        
		std::vector<char> name_buffer;
        
        Linux(LinuxFastCall& func_call) : name_buffer( 1024 ), section(nullptr), section_stack(nullptr), func_call(func_call) {}
        
        void init( Computer &computer );
        
        //File without extension
        void load_file(const fs::path& file);
        
        ulong get_return_64();
        void set_param1_32(uint p);
        void set_param1_64(ulong p);
        void set_param2_32(uint p);
        void set_param2_64(ulong p);
        void set_param3_32(uint p);
        void set_param3_64(ulong p);
        void set_param1_double(double p);

        void set_return_64(ulong r);
        ulong get_param1_64();
        ulong get_param2_64();

        bool uses_shadow_space() {
            return false;
        }
    };
}
