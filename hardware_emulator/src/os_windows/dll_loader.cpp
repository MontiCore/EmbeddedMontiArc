#include "os_windows\dll_loader.h"
#include <parser-library/parse.h>
#include "computer/system_calls.h"
using namespace std;
using namespace peparse;

int iter_exports( void *data, VA address, std::string &module_name, std::string &symbol_name ) {
    auto &loader = *static_cast<OS::DLLLoader *>( data );
    SysCall call( symbol_name, module_name, address );
    loader.sys_calls->add_syscall( call );
    return 0;
}
int iter_imports( void *data, VA address, const std::string &module_name, const std::string &symbol_name ) {
    auto &loader = *static_cast<OS::DLLLoader *>( data );
    SysCall call( symbol_name, module_name, nullptr );
    auto existing_call = loader.sys_calls->get_syscall( module_name, symbol_name );
    if ( existing_call == 0 )
        existing_call = loader.sys_calls->add_syscall( call );
    loader.mem->write_long_word( address, existing_call );
    return 0;
}
int iter_sections( void *data, VA secBase, std::string &name, image_section_header header, bounded_buffer *b ) {
    auto &loader = *static_cast<OS::DLLLoader *>( data );
    auto &sec = loader.mem->new_section();
    sec.init( MemoryRange( secBase, header.Misc.VirtualSize ), name, loader.file_name,
              header.Characteristics & IMAGE_SCN_MEM_EXECUTE,
              header.Characteristics & IMAGE_SCN_MEM_READ,
              header.Characteristics & IMAGE_SCN_MEM_WRITE );
    sec.set_file_range( MemoryRange( header.PointerToRawData, header.SizeOfRawData ) );
    sec.upload( loader.file.begin() + header.PointerToRawData,
                header.SizeOfRawData < header.Misc.VirtualSize ? header.SizeOfRawData : header.Misc.VirtualSize );
    return 0;
}

bool OS::DLLLoader::init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem ) {
    if ( loaded() )
        return true;
        
    this->sys_calls = &sys_calls;
    this->mem = &mem;
    this->file_name = file_name;
    
    FileReader fr;
    if ( !fr.open( file_name ) ) {
        cout << "Could not open DLL: " << file_name << endl;
        return false;
    }
    
    fr.read( file );
    pe = ParsePEFromMemory( file.begin(), file.size() );
    if ( pe == NULL ) {
        std::cout << "Error: " << GetPEErr() << " (" << GetPEErrString() << ")" << "\n";
        std::cout << "Location: " << GetPEErrLoc() << "\n";
        return false;
    }
    
    info.load_values( pe );
    
    IterSec( static_cast<peparse::parsed_pe *>( pe ), iter_sections, this );
    IterImpVAString( static_cast<peparse::parsed_pe *>( pe ), iter_imports, this );
    IterExpVA( static_cast<peparse::parsed_pe *>( pe ), iter_exports, this );
    
    file.drop();
    
    return true;
}

void OS::DLLLoader::drop() {
    if ( pe ) {
        DestructParsedPE( static_cast<peparse::parsed_pe *>( pe ) );
        pe = nullptr;
    }
}

void OS::DLLLoader::dll_main( Computer &computer ) {
    throw_assert( loaded(), "DLLLoader::dll_main() on uninitialized DLLLoader." );
    computer.fast_call.arg3.set_params( 0, 1, 0 ); //DLL_PROCESS_ATTACH
    computer.call( info.base_address + info.entry_point );
}

#define GET_H(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader. VarName)
#define GET_H64(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader64. VarName)

void OS::DLLInfo::load_values( void *pe ) {
    if ( static_cast<peparse::parsed_pe *>( pe )->peHeader.nt.OptionalMagic == NT_OPTIONAL_32_MAGIC ) {
        base_address = GET_H( pe, ImageBase );
        image_size = GET_H( pe, SizeOfImage );
        base_of_code = GET_H( pe, BaseOfCode );
        entry_point = GET_H( pe, AddressOfEntryPoint );
        section_align = GET_H( pe, SectionAlignment );
    }
    else {
        base_address = GET_H64( pe, ImageBase );
        image_size = GET_H64( pe, SizeOfImage );
        base_of_code = GET_H64( pe, BaseOfCode );
        entry_point = GET_H64( pe, AddressOfEntryPoint );
        section_align = GET_H64( pe, SectionAlignment );
    }
}
