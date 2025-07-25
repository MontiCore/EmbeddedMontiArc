/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "dll_loader.h"
#include "computer/os.h"
#include "computer/function_calling.h"
#include "windows_fast_call.h"

namespace OS {

    
    
    /*
        The Windows component sets up the Windows system function, the WindowsFastCall implementation and various
        objects needed by autopilots running under Windows.
    */
    struct Windows : public OS {
        WindowsFastCall& func_call;
        MemorySection *section = nullptr;
        SectionStack *section_stack = nullptr;
        
        DllLoader dll;
        
        MemoryRange cmd_line_wstr;
        MemoryRange cmd_line_str;
        
        MemoryRange io_stdin;
        MemoryRange io_stdout;
        MemoryRange io_stderr;

        MemoryRange lconv_slot;
        
		std::vector<char> name_buffer;
        
        Windows(WindowsFastCall& func_call) : name_buffer( 1024 ), func_call(func_call), dll(func_call) {}
        
        void init( Computer &computer );
        void setup_tib_gdt();
        void setup_command_line_args();
        void setup_cout();
        void setup_io();
        void setup_locale();
        
        //File without extension
        void load_file(const fs::path& file);
        

        ulong upload_system_string(const std::string& str, const char* description);


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
            return true;
        }
    };
    
}
