/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "os_linux.h"

#include "linux_system_calls.h"

namespace OS {

    void Linux::init( Computer &computer ) {
        this->computer = &computer;
        LinuxSystemCalls::add_linux_calls( computer.sys_calls );
    }
    
    
    void Linux::load_file( const fs::path &file ) {
        elf.init(file.parent_path() / (file.stem().string() + ".so"), computer->sys_calls, computer->memory, computer->symbols);
        elf.elf_main( *computer );
    }

    ulong Linux::get_return_64()
    {
        return func_call.get_return_64();
    }

    void Linux::set_param1_32(uint p)
    {
        func_call.set_param1_32(p);
    }

    void Linux::set_param1_64(ulong p)
    {
        func_call.set_param1_64(p);
    }

    void Linux::set_param2_32(uint p)
    {
        func_call.set_param2_32(p);
    }

    void Linux::set_param2_64(ulong p)
    {
        func_call.set_param2_64(p);
    }

    void Linux::set_param3_32(uint p)
    {
        func_call.set_param3_32(p);
    }

    void Linux::set_param3_64(ulong p)
    {
        func_call.set_param3_64(p);
    }

    void Linux::set_param1_double(double p)
    {
        func_call.set_param1_double(p);
    }

    void Linux::set_return_64(ulong r)
    {
        func_call.set_return_64(r);
    }

    ulong Linux::get_param1_64()
    {
        return func_call.get_param1_64();
    }

    ulong Linux::get_param2_64()
    {
        return func_call.get_param2_64();
    }
    
    
    
    
}
