#include "os_windows/dll_loader.h"
#include <parser-library/parse.h>
#include "computer/system_calls.h"
using namespace std;
using namespace peparse;

#define GET_H(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.FileHeader. VarName)
#define GET_OH(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader. VarName)
#define GET_OH64(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader64. VarName)

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
                
    auto &sec_info = loader.sections[loader.section_pos++];
    sec_info.mem = &sec;
    
    return 0;
}

int iter_symbols( void *data, std::string &str_name, uint32_t &value, int16_t &section_number,
                  uint16_t &type, uint8_t &storage_class, uint8_t &number_of_aux_symbols ) {
    auto &loader = *static_cast<OS::DLLLoader *>( data );
    if ( section_number < 1 )
        return 0;
    auto &sec = loader.sections[section_number - 1];
    auto &mem = *sec.mem;
    if ( !mem.has_annotations() )
        mem.init_annotations();
    mem.annotations.add_annotation( mem.address_range.start_address + value, Annotation( str_name, Annotation::SYMBOL ) );
    return 0;
}

bool OS::DLLLoader::init( const std::string &file_name, SystemCalls &sys_calls, Memory &mem ) {
    drop();
    
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
        file.drop();
        return false;
    }
    
    info.load_values( pe );
    
    sections.init( GET_H( pe, NumberOfSections ) );
    section_pos = 0;
    
    //Upload DLL Header
    auto &sec = mem.new_section();
    sec.init( MemoryRange( info.base_address, info.size_of_headers ), "DLL Header", file_name,
              false, true, false );
    sec.set_file_range( MemoryRange( 0, info.size_of_headers ) );
    sec.upload( file.begin(), info.size_of_headers );
    
    IterSec( static_cast<peparse::parsed_pe *>( pe ), iter_sections, this );
    IterImpVAString( static_cast<peparse::parsed_pe *>( pe ), iter_imports, this );
    IterExpVA( static_cast<peparse::parsed_pe *>( pe ), iter_exports, this );
    IterSymbols( static_cast<peparse::parsed_pe *>( pe ), iter_symbols, this );
    
    file.drop();
    
    return true;
}

void OS::DLLLoader::drop() {
    if ( loaded() ) {
        DestructParsedPE( static_cast<peparse::parsed_pe *>( pe ) );
        pe = nullptr;
    }
}

void OS::DLLLoader::dll_main( Computer &computer ) {
    throw_assert( loaded(), "DLLLoader::dll_main() on uninitialized DLLLoader." );
    computer.fast_call.set_params( 0x18C, 1, 0x10C ); //DLL_PROCESS_ATTACH
    computer.call( info.base_address + info.entry_point );
}



void OS::DLLInfo::load_values( void *pe ) {
    if ( static_cast<peparse::parsed_pe *>( pe )->peHeader.nt.OptionalMagic == NT_OPTIONAL_32_MAGIC ) {
        base_address = GET_OH( pe, ImageBase );
        image_size = GET_OH( pe, SizeOfImage );
        base_of_code = GET_OH( pe, BaseOfCode );
        entry_point = GET_OH( pe, AddressOfEntryPoint );
        section_align = GET_OH( pe, SectionAlignment );
        size_of_headers = GET_OH( pe, SizeOfHeaders );
    }
    else {
        base_address = GET_OH64( pe, ImageBase );
        image_size = GET_OH64( pe, SizeOfImage );
        base_of_code = GET_OH64( pe, BaseOfCode );
        entry_point = GET_OH64( pe, AddressOfEntryPoint );
        section_align = GET_OH64( pe, SectionAlignment );
        size_of_headers = GET_OH64( pe, SizeOfHeaders );
    }
}
