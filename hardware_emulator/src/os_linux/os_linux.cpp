/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "os_linux.h"

#include "linux_calls.h"

namespace OS {

    void Linux::init( Computer &computer ) {
        this->computer = &computer;
        LinuxCalls::add_linux_calls( computer.sys_calls );
        computer.func_call = std::unique_ptr<FunctionCalling>( new LinuxFastCall( computer.registers ) );
    }
    
    
    bool Linux::load_file( const char *file ) {
        if ( !elf.init( std::string( file ) +  ".so", computer->sys_calls, computer->memory, computer->symbols ) )
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
    void LinuxFastCall::set_params_double( double p1 ) {
        set_param1_double( p1 );
    }
    void LinuxFastCall::set_params_double( double p1, double p2 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
    }
    void LinuxFastCall::set_params_double( double p1, double p2, double p3 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
        set_param3_double( p3 );
    }
    void LinuxFastCall::set_params_double( double p1, double p2, double p3, double p4 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
        set_param3_double( p3 );
        set_param4_double( p4 );
    }
    void LinuxFastCall::set_params_float( float p1 ) {
        set_param1_float( p1 );
    }
    void LinuxFastCall::set_params_float( float p1, float p2 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
    }
    void LinuxFastCall::set_params_float( float p1, float p2, float p3 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
        set_param3_float( p3 );
    }
    void LinuxFastCall::set_params_float( float p1, float p2, float p3, float p4 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
        set_param3_float( p3 );
        set_param4_float( p4 );
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
        registers.set_rcx( p );
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
        registers.set_rcx( p );
    }
    void LinuxFastCall::set_param1_double( double p ) {
        registers.set_xmm0( p );
    }
    void LinuxFastCall::set_param2_double( double p ) {
        registers.set_xmm1( p );
    }
    void LinuxFastCall::set_param3_double( double p ) {
        registers.set_xmm2( p );
    }
    void LinuxFastCall::set_param4_double( double p ) {
        registers.set_xmm3( p );
    }
    void LinuxFastCall::set_param1_float( float p ) {
        registers.set_xmm0_f( p );
    }
    void LinuxFastCall::set_param2_float( float p ) {
        registers.set_xmm1_f( p );
    }
    void LinuxFastCall::set_param3_float( float p ) {
        registers.set_xmm2_f( p );
    }
    void LinuxFastCall::set_param4_float( float p ) {
        registers.set_xmm3_f( p );
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
    
    float LinuxFastCall::get_return_float() {
        return registers.get_xmm0_f();
    }
    
    char LinuxFastCall::get_return_char() {
        return ( char )registers.get_rax();
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
        return registers.get_rcx();
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
        return ( uint ) registers.get_rcx();
    }
    double LinuxFastCall::get_param1_double() {
        return registers.get_xmm0();
    }
    double LinuxFastCall::get_param2_double() {
        return registers.get_xmm1();
    }
    double LinuxFastCall::get_param3_double() {
        return registers.get_xmm2();
    }
    double LinuxFastCall::get_param4_double() {
        return registers.get_xmm3();
    }
    float LinuxFastCall::get_param1_float() {
        return registers.get_xmm0_f();
    }
    float LinuxFastCall::get_param2_float() {
        return registers.get_xmm1_f();
    }
    float LinuxFastCall::get_param3_float() {
        return registers.get_xmm2_f();
    }
    float LinuxFastCall::get_param4_float() {
        return registers.get_xmm3_f();
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