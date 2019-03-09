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
    
    
    //Caller
    void LinuxFastCall::set_params_64( ulong p1 ) {
        set_param1_64( p1 );
    }
    void LinuxFastCall::set_params_64( ulong p1, ulong p2 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
    }
    void LinuxFastCall::set_params_64( ulong p1, ulong p2, ulong p3 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
        set_param3_64( p3 );
    }
    void LinuxFastCall::set_params_64( ulong p1, ulong p2, ulong p3, ulong p4 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
        set_param3_64( p3 );
        set_param4_64( p4 );
    }
    void LinuxFastCall::set_params_32( uint p1 ) {
        set_param1_32( p1 );
    }
    void LinuxFastCall::set_params_32( uint p1, uint p2 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
    }
    void LinuxFastCall::set_params_32( uint p1, uint p2, uint p3 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
        set_param3_32( p3 );
    }
    void LinuxFastCall::set_params_32( uint p1, uint p2, uint p3, uint p4 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
        set_param3_32( p3 );
        set_param4_32( p4 );
    }
    void LinuxFastCall::set_param1_64( ulong p ) {
        registers.set_rdi( p );
    }
    void LinuxFastCall::set_param2_64( ulong p ) {
        registers.set_rsi( p );
    }
    void LinuxFastCall::set_param3_64( ulong p ) {
        registers.set_rdx( p );
    }
    void LinuxFastCall::set_param4_64( ulong p ) {
        Log::err << "TODO set_param4_64() LinuxFastCall\n";
    }
    void LinuxFastCall::set_param1_32( uint p ) {
        registers.set_rdi( p );
    }
    void LinuxFastCall::set_param2_32( uint p ) {
        registers.set_rsi( p );
    }
    void LinuxFastCall::set_param3_32( uint p ) {
        registers.set_rdx( p );
    }
    void LinuxFastCall::set_param4_32( uint p ) {
        Log::err << "TODO set_param4_32() LinuxFastCall\n";
    }
    void LinuxFastCall::set_param1_double( double p ) {
        registers.set_xmm0( p );
    }
    void LinuxFastCall::set_param2_double( double p ) {
        Log::err << "TODO set_param2_double() LinuxFastCall\n";
    }
    void LinuxFastCall::set_param3_double( double p ) {
        Log::err << "TODO set_param3_double() LinuxFastCall\n";
    }
    void LinuxFastCall::set_param4_double( double p ) {
        Log::err << "TODO set_param4_double() LinuxFastCall\n";
    }
    ulong LinuxFastCall::get_return_64() {
        return registers.get_rax();
    }
    uint LinuxFastCall::get_return_32() {
        return ( uint )registers.get_rax();
    }
    double LinuxFastCall::get_return_double() {
        return registers.get_xmm0();
    }
    
    
    
    
    //Callee
    ulong LinuxFastCall::get_param1_64() {
        return registers.get_rdi();
    }
    ulong LinuxFastCall::get_param2_64() {
        return registers.get_rsi();
    }
    ulong LinuxFastCall::get_param3_64() {
        return registers.get_rdx();
    }
    ulong LinuxFastCall::get_param4_64() {
        Log::err << "TODO get_param4_64() LinuxFastCall\n";
        return 0;
    }
    uint LinuxFastCall::get_param1_32() {
        return ( uint ) registers.get_rdi();
    }
    uint LinuxFastCall::get_param2_32() {
        return ( uint )registers.get_rsi();
    }
    uint LinuxFastCall::get_param3_32() {
        return ( uint )registers.get_rdx();
    }
    uint LinuxFastCall::get_param4_32() {
        Log::err << "TODO get_param4_32() LinuxFastCall\n";
        return 0;
    }
    double LinuxFastCall::get_param1_double() {
        return registers.get_xmm0();
    }
    double LinuxFastCall::get_param2_double() {
        Log::err << "TODO get_param2_double() LinuxFastCall\n";
        return 0.0;
    }
    double LinuxFastCall::get_param3_double() {
        Log::err << "TODO get_param3_double() LinuxFastCall\n";
        return 0.0;
    }
    double LinuxFastCall::get_param4_double() {
        Log::err << "TODO get_param4_double() LinuxFastCall\n";
        return 0.0;
    }
    void LinuxFastCall::set_return_64( ulong r ) {
        registers.set_rax( r );
    }
    void LinuxFastCall::set_return_32( uint r ) {
        registers.set_rax( r );
    }
    void LinuxFastCall::set_return_double( double r ) {
        registers.set_xmm0( r );
    }
    
    
    
}