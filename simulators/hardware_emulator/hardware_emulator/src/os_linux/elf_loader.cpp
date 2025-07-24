/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "elf_loader.h"
#include <unordered_map>
#include <iostream> //TEMP


using namespace std;

void OS::ElfLoader::init(const fs::path& fn, SystemCalls &sys_calls, Memory &mem, Symbols &symbols ) {
    loaded = false;
    this->sys_calls = &sys_calls;
    this->mem = &mem;
    this->symbols = &symbols;
    file_name = fn.filename().string();
    
    module_name = "SYSTEM";
    
    FileReader fr;
    if ( !fr.open( fn ) )
        throw_error(Error::hardware_emu_software_load_error("[ElfLoader] Could not find software program: " + fn.string()));
    
    
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
        //entry_point_address = elf.header64().e_entry;
    }
    else {
        for ( auto &seg : elf.ph32 )
            if ( seg.get_type() == ElfSegType::PT_LOAD )
                seg_count++;
        //entry_point_address = elf.header32().e_entry;
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

        auto& section_string_table_section = elf.sh64[elf.header64().e_shstrndx];
        auto section_string_table = elf.get_section_as_table<char>(section_string_table_section);
        
        //Log::info << "Resolving imports\n";
        for ( auto &sh : elf.sh64 ) {
            if ( sh.get_type() == ElfSecType::SHT_RELA ) {
                std::string table_name = section_string_table.begin() + sh.sh_name;
                cout << "Reloc section name: " << table_name << endl;
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
                        if (name == "_ZNSt6locale18_S_initialize_onceEv") {
                            int a = 0;
                        }
                        
                        auto sym = symbols.get_symbol( name );
                        if ( sym.type == Symbols::Symbol::Type::OBJECT || sym.type == Symbols::Symbol::Type::EXPORT)
                            mem.write_long_word( r.r_offset, sym.addr );
                        else cerr << "Un-relocatable Object " << name << "\n";
                    }
                }
            }
        }

        Elf64_Dyn *dt_init_entry = nullptr;
        // load dynamic section for initialization functions
        // All the following contain VIRTUAL addresses
        for ( auto &sh : elf.sh64 ) {
            if ( sh.get_type() == ElfSecType::SHT_DYNAMIC) {
                if (!sh.has_flag(ElfSecFlags::SHF_ALLOC)) {
                    std::cerr << "Expected ALLOC flag" << std::endl;
                }
                auto dyn_table = elf.get_section_as_table<Elf64_Dyn>( sh );
                
                for ( auto &dyn_entry : dyn_table ) {
                    auto t = dyn_entry.get_type();
                    if (t == ElfDynType::DT_INIT) {
                        dt_init_entry = &dyn_entry;
                    } else if (t == ElfDynType::DT_INIT_ARRAY) {
                        dt_init_array_entry = &dyn_entry;
                    } else if (t == ElfDynType::DT_INIT_ARRAYSZ) {
                        dt_init_array_size_entry = &dyn_entry;
                    }
                }
            }
        }

        if (dt_init_entry) {
            init_address = dt_init_entry->d_un.d_ptr;
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
    //computer.debug.d_code = true;// TEMP
    //computer.debug.d_mem = true;// TEMP
    //computer.debug.d_reg_update = true;// TEMP

    
    computer.call( init_address, ".init section call" );

    // TODO temporarily removed, but be fixed
    // if (dt_init_array_entry && dt_init_array_size_entry) {

    //     if ( elf.is_64bit ) {
    //         auto addrs = (Elf64_Addr *) (dt_init_array_entry->d_un.d_ptr); // !! virtual address
    //         auto res = (Elf64_Addr *) computer.memory.read_memory(MemoryRange((ulong)addrs, dt_init_array_size_entry->d_un.d_val));
    //         auto jm = dt_init_array_size_entry->d_un.d_val / sizeof (Elf64_Addr);
    //         for (auto j = 0; j < jm; ++j) {
    //             auto addr = res[j];
    //             if (addr != 0 && addr != -1) {
    //                 computer.call(addr, ".init_array section call" );
    //             }
    //         }
    //     } else {
    //         throw_assert( false, "TODO" );
    //     }

    // }

    // auto init = symbols->get_symbol( "_init" );
    // if ( init.type == Symbols::Symbol::Type::EXPORT ) {
    //     //computer.call( init.addr, "_init" );
    //     computer.call( entry_point_address, "ELF ENTRY POINT" );
        
    // }
    // else
    //     throw_error(Error::hardware_emu_software_load_error("[ElfLoader] Could not locate _init function of ELF."));
}
