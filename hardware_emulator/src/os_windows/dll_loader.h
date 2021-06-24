/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <string>
#include "utility/utility.h"
#include "computer/system_calls.h"
#include "computer/memory.h"
#include "computer/computer.h"
#include "computer/os.h"
#include "computer/symbols.h"
#include "windows_fast_call.h"

namespace OS {



    /*
        This component is responsible for loading a DLL file into the computer memory
        and expose the symbols (functions) of the file.
    
        dll_main() calls the entry point of DLLs.
    */
    struct DllLoader {
        void *pe;
		std::vector<char> file;
        std::string file_name;
        std::string module_name;
        bool module_name_set = false;
        
        SystemCalls *sys_calls;
        Symbols *symbols;
        Memory *mem;
        WindowsFastCall &func_call;
        
        struct DLLInfo {
            ulong base_address = 0;
            ulong image_size = 0;
            ulong base_of_code = 0;
            ulong entry_point = 0;
            ulong section_align = 0;
            uint size_of_headers = 0;
            
            void load_values( void *pe );
        } info;
        
        
        
		std::vector<SectionInfo> sections;
        uint section_pos = 0;
        
        DllLoader(WindowsFastCall& func_call) : pe( nullptr ), sys_calls( nullptr ), mem( nullptr ), symbols( nullptr ), func_call(func_call) {}
        ~DllLoader() {
            drop();
        }
        
        //File without extension
        void init(const fs::path& file, SystemCalls &sys_calls, Memory &mem, Symbols &symbols );
        void drop();
        bool loaded() {
            return pe != nullptr;
        }
        
        void dll_main( Computer &computer );
        
        
    };
    
}
