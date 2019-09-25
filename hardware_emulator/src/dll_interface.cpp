/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
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
    addresses.resize( FUNCTION_COUNT );
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
    addresses.resize( FUNCTION_COUNT );
    if ( !add_symbol( addresses[TEST_METHOD], "test_method", computer ) )
        return false;
    return true;
}
void LOADED_DLL::Interface::test_method() {
    call_success = computer->call( addresses[TEST_METHOD], "test_method" );
}


