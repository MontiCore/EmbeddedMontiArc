#include "dll_interface.h"
#include "utility/utility.h"

using namespace std;

void add_symbol( uint64_t &target, const std::string &name, Computer &computer ) {
    auto add_sym = computer.symbols.get_symbol( name );
    if (add_sym.type == Symbols::Symbol::Type::NONE)
        throw_error(Error::hardware_emu_software_load_error("Cannot find symbol \"" + name + "\" in DLL."));
    target = add_sym.addr;
}

void ADD_DLL::Interface::init( Computer &computer, bool windows ) {
    TestProgramInterface::init( computer );
    computer.os->load_file(FS::File("sample_simple"));
    addresses.resize( FUNCTION_COUNT );
    add_symbol(addresses[ADD], windows ? "add" : "add", computer);
}

int ADD_DLL::Interface::add( int a, int b ) {
    computer->func_call->set_params_32( *( ( uint32_t * )&a ), *( ( uint32_t * )&b ) );
    computer->call(addresses[ADD], "add");
    auto res = computer->func_call->get_return_32();
    return *( ( int * ) & ( res ) );
}



void LOADED_DLL::Interface::init( Computer &computer ) {
    TestProgramInterface::init( computer );
    computer.os->load_file(FS::File("sample_syscall"));
    addresses.resize( FUNCTION_COUNT );
    add_symbol(addresses[TEST_METHOD], "test_method", computer);
}
void LOADED_DLL::Interface::test_method() {
    computer->call(addresses[TEST_METHOD], "test_method");
}


