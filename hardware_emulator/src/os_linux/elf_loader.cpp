/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "elf_loader.h"
#include <unordered_map>


using namespace std;

void OS::ElfLoader::init(const FS::File& fn, SystemCalls &sys_calls, Memory &mem, Symbols &symbols ) {
    loaded = false;
    this->sys_calls = &sys_calls;
    this->mem = &mem;
    this->symbols = &symbols;
    file_name = fn.get_full_name();
    
    module_name = "SYSTEM";
    
    FileReader fr;
    if ( !fr.open( fn ) )
        throw_error(Error::hardware_emu_software_load_error("[ElfLoader] Could not find software program: " + fn.as_system_path()));
    
    
    fr.read( elf.data );
    
    if ( !elf.parse() )
        throw_error(Error::hardware_emu_software_load_error("[ElfLoader] Error parsing software program."));
        
    //elf.print();

    //info.load_values( &elf );
    uint seg_count = 0;
    //Count segments in memory
    if ( elf.is_64bit ) {
        for ( auto &seg : elf.ph64 )
            if ( seg.get_type() == ElfSegType::PT_LOAD )
                seg_count++;
    }
    else {
        for ( auto &seg : elf.ph32 )
            if ( seg.get_type() == ElfSegType::PT_LOAD )
                seg_count++;
                
    }
    sections.resize( seg_count );
    section_pos = 0;
    //Setup segments
    
    if ( elf.is_64bit ) {
        //Log::info << "Loading segments\n";
        for ( auto &seg : elf.ph64 ) {
            if ( seg.get_type() == ElfSegType::PT_LOAD ) {
                if ( seg.p_memsz == 0 )
                    continue;
                auto &sec = mem.new_section(
                                MemoryRange( seg.p_vaddr, ( uint )seg.p_memsz ), "seg" + std::to_string( section_pos ),
                                file_name,
                                seg.has_perm( Elf32_ProgramPerm::PF_X ),
                                seg.has_perm( Elf32_ProgramPerm::PF_R ),
                                seg.has_perm( Elf32_ProgramPerm::PF_W ) );
                                
                                
                if ( seg.p_filesz > 0 ) {
                    sec.set_mapped_range( MemoryRange( seg.p_vaddr, ( uint )seg.p_filesz ), seg.p_offset );
                    sec.upload( MemoryRange( seg.p_vaddr, ( uint )seg.p_filesz ), ( char * ) ( elf.data.data() + seg.p_offset ) );
                }
                
                auto &sec_info = sections[section_pos++];
                sec_info.mem = &sec;
            }
        }
        //Log::info << "Reading symbols\n";
        for ( auto &sh : elf.sh64 ) {
            if ( sh.get_type() == ElfSecType::SHT_DYNSYM || sh.get_type() == ElfSecType::SHT_SYMTAB) {
                auto sym_sh = elf.get_section_as_table<Elf64_Symbol>( sh );
                auto &str_sh = elf.sh64[sh.sh_link];
                auto str_table = elf.get_section_as_table<char>( str_sh );
                
                for ( auto &sym : sym_sh ) {
                    auto t = sym.get_type();
                    if ( sym.st_value != 0 && ( t == ElfSymbolType::STT_FUNC || t == ElfSymbolType::STT_OBJECT ) ) {
                        auto type = t == ElfSymbolType::STT_FUNC ? Symbols::Symbol::Type::EXPORT : Symbols::Symbol::Type::OBJECT;
                        symbols.add_symbol( str_table.begin() + sym.st_name, type, sym.st_value );
                        
                        auto sec = mem.get_section( sym.st_value );
                        if ( sec ) {
                            auto &mem = *sec;
                            if ( !mem.has_annotations() )
                                mem.init_annotations();
                            mem.annotations.add_annotation( sym.st_value, Annotation( str_table.begin() + sym.st_name, Annotation::Type::OBJECT ) );
                        }
                    }
                }
            }
        }
        
        //Log::info << "Resolving imports\n";
        for ( auto &sh : elf.sh64 ) {
            if ( sh.get_type() == ElfSecType::SHT_RELA ) {
                //Relocation table
                auto rela = elf.get_section_as_table<Elf64_Rela>( sh );
                
                //Corresponding symbol table
                auto &sym_sh = elf.sh64[sh.sh_link];
                auto sym = elf.get_section_as_table<Elf64_Symbol>( sym_sh );
                //Symbol table string table
                auto sym_str = elf.get_section_as_table<char>( elf.sh64[sym_sh.sh_link] );
                
                //Target section
                auto &target = elf.sh64[sh.sh_info];
                for ( auto j : urange( rela.size() ) ) {
                    auto &r = rela[j];
                    if ( r.get_type() == ElfRelType::R_386_JMP_SLOT ) {
                        //Log::info << "Reloc Func: ";
                        auto &s = sym[r.get_sym_raw()];
                        std::string name = sym_str.begin() + s.st_name;
                        
                        ulong func_addr;
                        
                        auto sym = symbols.get_symbol( name );
                        if ( sym.type != Symbols::Symbol::Type::NONE )
                            func_addr = sym.addr;
                        else
                            func_addr = sys_calls.add_syscall( SysCall( name, "", nullptr ), "elf resolve" );
                        mem.write_long_word( r.r_offset, func_addr );
                    }
                    if ( r.get_type() == ElfRelType::R_386_GLOB_DAT ) {
                        //Log::info << "Reloc Object: ";
                        auto &s = sym[r.get_sym_raw()];
                        std::string name = sym_str.begin() + s.st_name;
                        
                        auto sym = symbols.get_symbol( name );
                        if ( sym.type == Symbols::Symbol::Type::OBJECT )
                            mem.write_long_word( r.r_offset, sym.addr );
                        //else Log::err << Log::tag << "Un-relocatable Object " << name << "\n";
                    }
                }
            }
        }
    }
    else {
        //TODO
    }
    
    elf.data.clear();
    //No more access to any ElfFile elements
    loaded = true;
}

void OS::ElfLoader::elf_main( Computer &computer ) {
    throw_assert( loaded, "ElfLoader::elf_main() on uninitialized ElfLoader." );
    auto init = symbols->get_symbol( "_init" );
    if ( init.type == Symbols::Symbol::Type::EXPORT )
        computer.call( init.addr, "_init" );
    else
        throw_error(Error::hardware_emu_software_load_error("[ElfLoader] Could not locate _init function of ELF."));
}
