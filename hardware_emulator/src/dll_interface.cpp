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
#include "dll_interface.h"
#include "utility.h"

using namespace std;

bool add_symbol( uint64_t &target, const std::string &name, Computer &computer ) {
    auto add_sym = computer.symbols.get_symbol( name );
    if ( add_sym.type == Symbols::Symbol::NONE )
        return false;
    target = add_sym.addr;
    return true;
}

bool ADD_DLL::Interface::init( Computer &computer, bool windows ) {
    ProgramInterface::init( computer );
    if ( !computer.os->load_file( "sample_simple" ) )
        return false;
    addresses.init( FUNCTION_COUNT );
    if ( !add_symbol( addresses[ADD], windows ? "add" : "add", computer ) )
        return false;
    return true;
}

int ADD_DLL::Interface::add( int a, int b ) {
    computer->func_call->set_params_32( *( ( uint32_t * )&a ), *( ( uint32_t * )&b ) );
    call_success = computer->call( addresses[ADD], "add" );
    auto res = computer->func_call->get_return_32();
    return *( ( int * ) & ( res ) );
}



bool LOADED_DLL::Interface::init( Computer &computer ) {
    ProgramInterface::init( computer );
    if ( !computer.os->load_file( "sample_syscall" ) )
        return false;
    addresses.init( FUNCTION_COUNT );
    if ( !add_symbol( addresses[TEST_METHOD], "test_method", computer ) )
        return false;
    return true;
}
void LOADED_DLL::Interface::test_method() {
    call_success = computer->call( addresses[TEST_METHOD], "test_method" );
}


