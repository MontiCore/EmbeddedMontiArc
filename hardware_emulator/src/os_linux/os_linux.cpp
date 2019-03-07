#include "os_linux.h"

#include "linux_calls.h"

namespace OS {

    void Linux::init( Computer &computer ) {
        this->computer = &computer;
        LinuxCalls::add_linux_calls( computer.sys_calls );
        computer.func_call = std::unique_ptr<FunctionCalling>( new LinuxFastCall( computer.registers ) );
    }
    
    
    bool Linux::load_file( const char *file ) {
        if ( !elf.init( file, computer->sys_calls, computer->memory, computer->symbols ) )
            return false;
        elf.elf_main( *computer );
        
        return true;
    }
    
    
    
    void LinuxFastCall::set_params( ulong p1 ) {
        registers.set_rdi( p1 );
    }
    
    void LinuxFastCall::set_params( ulong p1, ulong p2 ) {
        registers.set_rdi( p1 );
        registers.set_rsi( p2 );
    }
    
    void LinuxFastCall::set_params( ulong p1, ulong p2, ulong p3 ) {
        registers.set_rdi( p1 );
        registers.set_rsi( p2 );
        registers.set_rdx( p3 );
    }
    
    void LinuxFastCall::set_params( ulong p1, ulong p2, ulong p3, ulong p4 ) {
        Log::err << "TODO set_params() LinuxFastCall\n";
    }
    
    ulong LinuxFastCall::get_param1() {
        return registers.get_rdi();
    }
    
    ulong LinuxFastCall::get_param2() {
        return registers.get_rsi();
    }
    
    ulong LinuxFastCall::get_param3() {
        return registers.get_rdx();
    }
    
    ulong LinuxFastCall::get_param4() {
        Log::err << "TODO get_param4() LinuxFastCall\n";
        return 0;
    }
    
    ulong LinuxFastCall::get_return() {
        return registers.get_rax();
    }
    
    void LinuxFastCall::set_return( ulong r ) {
        registers.set_rax( r );
    }
    
}