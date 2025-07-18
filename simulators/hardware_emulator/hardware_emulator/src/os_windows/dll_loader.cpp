/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "os_windows/dll_loader.h"
#include <parser-library/parse.h>
#include "computer/system_calls.h"
using namespace std;
using namespace peparse;

#define GET_H(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.FileHeader. VarName)
#define GET_OH(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader. VarName)
#define GET_OH64(PE, VarName) static_cast<ulong>(static_cast<peparse::parsed_pe *> (PE)->peHeader.nt.OptionalHeader64. VarName)

int iter_exports( void *data, VA address, std::string &module_name, std::string &symbol_name ) {
    auto &loader = *static_cast<OS::DllLoader *>( data );
    loader.symbols->add_symbol( symbol_name, Symbols::Symbol::Type::EXPORT, address );
    return 0;
}
int iter_imports( void *data, VA address, const std::string &module_name, const std::string &symbol_name ) {
    auto &loader = *static_cast<OS::DllLoader *>( data );
    SysCall call( symbol_name, module_name, nullptr );
    ulong func_addr;
    auto sym = loader.symbols->get_symbol( symbol_name );
    if ( sym.type == Symbols::Symbol::Type::NONE )
        func_addr = loader.sys_calls->add_syscall( call, "dll resolve" );
    else
        func_addr = sym.addr;
    loader.mem->write_long_word( address, func_addr );
    return 0;
}
int iter_sections( void *data, VA secBase, std::string &name, image_section_header header, bounded_buffer *b ) {
    auto &loader = *static_cast<OS::DllLoader *>( data );
    auto &sec = loader.mem->new_section(
                    MemoryRange( secBase, header.Misc.VirtualSize ), name, loader.file_name,
                    header.Characteristics & IMAGE_SCN_MEM_EXECUTE,
                    header.Characteristics & IMAGE_SCN_MEM_READ,
                    header.Characteristics & IMAGE_SCN_MEM_WRITE
                );
    sec.set_mapped_range( MemoryRange( secBase, header.SizeOfRawData ), header.PointerToRawData );
    sec.upload( MemoryRange(
                    secBase,
                    header.SizeOfRawData < header.Misc.VirtualSize ? header.SizeOfRawData : header.Misc.VirtualSize
                ),
                loader.file.data() + header.PointerToRawData );
                
    auto &sec_info = loader.sections[loader.section_pos++];
    sec_info.mem = &sec;
    
    return 0;
}

int iter_symbols( void *data, std::string &str_name, uint32_t &value, int16_t &section_number,
                  uint16_t &type, uint8_t &storage_class, uint8_t &number_of_aux_symbols ) {
    auto &loader = *static_cast<OS::DllLoader *>( data );
    if ( section_number < 1 )
        return 0;
    auto &sec = loader.sections[section_number - 1LL];
    auto &mem = *sec.mem;
    if ( !mem.has_annotations() )
        mem.init_annotations();
    mem.annotations.add_annotation( mem.address_range.start_address + value, Annotation( str_name, Annotation::Type::OBJECT ) );
    return 0;
}

//File without extension
void OS::DllLoader::init(const fs::path& fn, SystemCalls &sys_calls, Memory &mem, Symbols &symbols ) {
    drop();
    
    this->sys_calls = &sys_calls;
    this->mem = &mem;
    this->symbols = &symbols;
    file_name = fn.filename().string();
    
    FileReader fr;
    if ( !fr.open(fn) )
        throw_error(Error::hardware_emu_software_load_error("[DllLoader] Could not find software program: " + fn.string()));
    
    fr.read( file );
    pe = ParsePEFromMemory( file.data(), (unsigned long) file.size() );
    if ( pe == NULL ) throw_error(Error::hardware_emu_software_load_error("[DllLoader] Error parsing software program: "
        + GetPEErrString() + "\n\tLocation: " + GetPEErrLoc()));
    
    info.load_values( pe );
    
    sections.resize( GET_H( pe, NumberOfSections ) );
    section_pos = 0;
    
    //Upload DLL Header
    auto &sec = mem.new_section( MemoryRange( info.base_address, info.size_of_headers ), "DLL Header", file_name,
                                 false, true, false );
    sec.set_mapped_range( MemoryRange( info.base_address, info.size_of_headers ), 0 );
    sec.upload( MemoryRange( info.base_address, info.size_of_headers ), file.data() );
    
    module_name_set = false;
    IterSec( static_cast<peparse::parsed_pe *>( pe ), iter_sections, this );
    IterImpVAString( static_cast<peparse::parsed_pe *>( pe ), iter_imports, this );
    IterExpVA( static_cast<peparse::parsed_pe *>( pe ), iter_exports, this );
    IterSymbols( static_cast<peparse::parsed_pe *>( pe ), iter_symbols, this );
    
    file.clear();
}

void OS::DllLoader::drop() {
    if ( loaded() ) {
        DestructParsedPE( static_cast<peparse::parsed_pe *>( pe ) );
        pe = nullptr;
    }
}

void OS::DllLoader::dll_main( Computer &computer ) {
    throw_assert( loaded(), "DllLoader::dll_main() on uninitialized DllLoader." );
    func_call.set_params_64( 0x18C, 1, 0x10C ); //DLL_PROCESS_ATTACH
    computer.call( info.base_address + info.entry_point, "dll_main" );
}



void OS::DllLoader::DLLInfo::load_values( void *pe ) {
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
