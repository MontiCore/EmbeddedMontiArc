/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "elf.h"
#include <inttypes.h>

ValueName elf_sec_type_name[static_cast<uint>( ElfSecType::SHT_NUM )] = {
    ValueName( ElfSecType::SHT_NULL,        "NULL    " ),
    ValueName( ElfSecType::SHT_PROGBITS,    "PROGBITS" ),
    ValueName( ElfSecType::SHT_SYMTAB,      "SYMTAB  " ),
    ValueName( ElfSecType::SHT_STRTAB,      "STRTAB  " ),
    ValueName( ElfSecType::SHT_RELA,        "RELA    " ),
    ValueName( ElfSecType::SHT_HASH,        "HASH    " ),
    ValueName( ElfSecType::SHT_DYNAMIC,     "DYNAMIC " ),
    ValueName( ElfSecType::SHT_NOTE,        "NOTE    " ),
    ValueName( ElfSecType::SHT_NOBITS,      "NOBITS  " ),
    ValueName( ElfSecType::SHT_REL,         "REL     " ),
    ValueName( ElfSecType::SHT_SHLIB,       "SHLIB   " ),
    ValueName( ElfSecType::SHT_DYNSYM,      "DYNSYM  " ),
    ValueName( ElfSecType::SHT_INIT_ARRAY,  "INIT_ARR" ),
    ValueName( ElfSecType::SHT_FINI_ARRAY,  "FINI_ARR" ),
    ValueName( ElfSecType::SHT_PREINIT_ARRAY, "PINI_ARR" ),
    ValueName( ElfSecType::SHT_GROUP,       "GROUP   " ),
    ValueName( ElfSecType::SHT_SYMTAB_SHNDX, "SYMTAB_S" ),
};

ValueName elf_sym_bind_name[static_cast<uint>( ElfSymbolBind::STB_NUM )] = {
    ValueName( ElfSymbolBind::STB_LOCAL,    "LOCAL " ),
    ValueName( ElfSymbolBind::STB_GLOBAL,   "GLOBAL" ),
    ValueName( ElfSymbolBind::STB_WEAK,     "WEAK  " ),
};
ValueName elf_sym_type_name[static_cast<uint>( ElfSymbolType::STT_NUM )] = {
    ValueName( ElfSymbolType::STT_NOTYPE,   "NOTYPE " ),
    ValueName( ElfSymbolType::STT_OBJECT,   "OBJECT " ),
    ValueName( ElfSymbolType::STT_FUNC,     "FUNC   " ),
    ValueName( ElfSymbolType::STT_SECTION,  "SECTION" ),
    ValueName( ElfSymbolType::STT_FILE,     "FILE   " ),
    ValueName( ElfSymbolType::STT_COMMON,   "COMMON " ),
    ValueName( ElfSymbolType::STT_TLS,      "TLS    " ),
};


ValueName elf_seg_type_name[static_cast<uint>( ElfSegType::PT_NUM )] = {
    ValueName( ElfSegType::PT_NULL,      "NULL   " ),
    ValueName( ElfSegType::PT_LOAD,      "LOAD   " ),
    ValueName( ElfSegType::PT_DYNAMIC,   "DYNAMIC" ),
    ValueName( ElfSegType::PT_INTERP,    "INTERP " ),
    ValueName( ElfSegType::PT_NOTE,      "NOTE   " ),
    ValueName( ElfSegType::PT_SHLIB,     "SHLIB  " ),
    ValueName( ElfSegType::PT_PHDR,      "PHDR   " ),
    ValueName( ElfSegType::PT_TLS,       "TLS    " ),
};

void hlp_print_line() {
    printf( "========================================" );
    printf( "========================================\n" );
}

const char *ElfMag::MAG = "\177ELF";

bool ElfFile::parse() {
    if ( data.size() < sizeof( ElfIdent ) )
        return false;
    ident_ptr = ( ElfIdent * )data.data();
    auto &id = ident();
    
    if ( !id.is_elf() )
        return false;
    is_64bit = id.get_class() == ElfClass::CLASS_64;
    if ( !is_64bit ) {
        if ( data.size() < sizeof( Elf32_Header ) )
            return false;
        header32_ptr = ( Elf32_Header * )data.data();
        auto &header = header32();
        
        //Load Section Header table
        if ( !header.section_header_size_valid() )
            return false;
        if ( header.e_shoff + header.e_shnum * (ulong) header.e_shentsize > data.size() )
            return false;
        sh32.init( ( Elf32_SectionHeader * )( data.data() + header.e_shoff ), 0, header.e_shnum );
        
        //Load Program Header table
        if ( !header.program_header_size_valid() )
            return false;
        if ( header.e_phoff + header.e_phnum * (ulong) header.e_phentsize > data.size() )
            return false;
        ph32.init( ( Elf32_ProgramHeader * )( data.data() + header.e_phoff ), 0, header.e_phnum );
        
        //Section name table
        auto &sec_names = sh32[header.e_shstrndx];
        sec_name_table.init( ( char * )data.data(), sec_names.sh_offset, sec_names.sh_size );
        
        //Load symbol table
        /*bool found = false;
        for ( auto &sh : sh32 ) {
            auto type = sh.get_type();
            if ( type == ElfSecType::SHT_SYMTAB || type == ElfSecType::SHT_DYNSYM ) {
                if ( found ) {
                    Log::err << "Multiple Symbol Tables found\n";
                    break;
                }
                found = true;
                sym32.init( ( Elf32_Symbol * )( data.data() + sh.sh_offset ), 0, sh.sh_size / sizeof( Elf32_Symbol ) );
                auto &str_sh = sh32[sh.sh_link];
                str_table.init( ( char * )data.data(), str_sh.sh_offset, str_sh.sh_size );
            }
        }*/
    }
    else {
        if ( data.size() < sizeof( Elf64_Header ) )
            return false;
        header64_ptr = ( Elf64_Header * )data.data();
        auto &header = header64();
        
        //Load section table
        if ( !header.section_header_size_valid() )
            return false;
        if ( header.e_shoff + header.e_shnum * (ulong) header.e_shentsize > data.size() )
            return false;
        sh64.init( ( Elf64_SectionHeader * )( data.data() + header.e_shoff ), 0, header.e_shnum );
        
        //Load Program Header table
        if ( !header.program_header_size_valid() )
            return false;
        if ( header.e_phoff + header.e_phnum * (ulong) header.e_phentsize > data.size() )
            return false;
        ph64.init( ( Elf64_ProgramHeader * )( data.data() + header.e_phoff ), 0, header.e_phnum );
        
        //Section name table
        auto &sec_names = sh64[header.e_shstrndx];
        sec_name_table.init( ( char * )data.data(), ( uint ) sec_names.sh_offset, ( uint )sec_names.sh_size );
        
        //Load symbol table
        /*bool found = false;
        for ( auto &sh : sh64 ) {
            auto type = sh.get_type();
            if ( type == ElfSecType::SHT_SYMTAB || type == ElfSecType::SHT_DYNSYM ) {
                if ( found ) {
                    Log::err << "Multiple Symbol Tables found\n";
                    break;
                }
                found = true;
                sym64.init( ( Elf64_Symbol * )( data.data() + sh.sh_offset ), 0, ( uint )( sh.sh_size / sizeof( Elf64_Symbol ) ) );
                auto &str_sh = sh64[sh.sh_link];
                str_table.init( ( char * )data.data(), ( uint )str_sh.sh_offset, ( uint )str_sh.sh_size );
            }
        }*/
    }
    
    
    return true;
}

void ElfFile::print() {
    if ( is_64bit )
        header64().print();
    else
        header32().print();
    print_sections();
    print_symbols();
    print_relocations();
    print_segments();
}

void ElfFile::print_sections() {
    hlp_print_line();
    printf( " idx offset     load-addr  size       algn"
            " flags      type       section\n" );
    hlp_print_line();
    if ( is_64bit ) {
        for ( auto i : urange( sh64.size() ) ) {
            printf( " %03d ", i );
            sh64[i].print( sec_name_table );
        }
    }
    else {
        for ( auto i : urange( sh32.size() ) ) {
            printf( " %03d ", i );
            sh32[i].print( sec_name_table );
        }
    }
    hlp_print_line();
    printf( "\n" );
}

void ElfFile::print_segments() {
    printf( "Segments\n" );
    hlp_print_line();
    printf( " idx  vaddr      mem-size   offset     file-size    align  flags      type\n" );
    hlp_print_line();
    if ( is_64bit ) {
        for ( auto i : urange( ph64.size() ) ) {
            printf( " %03d ", i );
            ph64[i].print();
        }
    }
    else {
        for ( auto i : urange( ph32.size() ) ) {
            printf( " %03d ", i );
            ph32[i].print();
        }
    }
    hlp_print_line();
    printf( "\n" );
}

void hlpsym_print_header() {
    hlp_print_line();
    printf( " value      size     shid bind   type    name\n" );
    hlp_print_line();
}




void ElfFile::print_symbols() {
    if ( is_64bit ) {
        for ( auto i : urange( sh64.size() ) ) {
            auto &sh = sh64[i];
            auto type = sh.get_type();
            if ( type == ElfSecType::SHT_SYMTAB || type == ElfSecType::SHT_DYNSYM ) {
                printf( "\n[Section %03d]", i );
                vector_slice<Elf64_Symbol> sym64;
                auto symbol_count = ( uint )( sh.sh_size / sizeof( Elf64_Symbol ) );
                sym64.init( ( Elf64_Symbol * )( data.data() + sh.sh_offset ), 0, symbol_count );
                
                vector_slice<char> str_table;
                auto &str_sh = sh64[sh.sh_link];
                str_table.init( ( char * )data.data(), ( uint )str_sh.sh_offset, ( uint )str_sh.sh_size );
                
                printf( " %d symbols\n", symbol_count );
                hlpsym_print_header();
                for ( auto j : urange( symbol_count ) ) {
                    auto &sym = sym64[j];
                    printf( " 0x%08" PRIx64 " ", sym.st_value );
                    printf( "%8" PRIx64 " ", sym.st_size );
                    printf( "%04" PRIx32 " ", sym.st_shndx );
                    printf( "%s ", elf_sym_bind_name[sym.get_bind_raw()].name );
                    printf( "%s ", elf_sym_type_name[sym.get_type_raw()].name );
                    printf( "%s\n", str_table.begin() + sym.st_name );
                }
                hlp_print_line();
            }
        }
    }
    else {
        for ( auto i : urange( sh32.size() ) ) {
            auto &sh = sh32[i];
            auto type = sh.get_type();
            if ( type == ElfSecType::SHT_SYMTAB || type == ElfSecType::SHT_DYNSYM ) {
                printf( "\n[Section %03d]", i );
                vector_slice<Elf32_Symbol> sym32;
                auto symbol_count = ( uint )( sh.sh_size / sizeof( Elf32_Symbol ) );
                sym32.init( ( Elf32_Symbol * )( data.data() + sh.sh_offset ), 0, symbol_count );
                
                vector_slice<char> str_table;
                auto &str_sh = sh32[sh.sh_link];
                str_table.init( ( char * )data.data(), ( uint )str_sh.sh_offset, ( uint )str_sh.sh_size );
                
                printf( " %d symbols\n", symbol_count );
                hlpsym_print_header();
                for ( auto j : urange( symbol_count ) ) {
                    auto &sym = sym32[j];
                    printf( " 0x%08" PRIx32 " ", sym.st_value );
                    printf( "%8" PRIx32 " ", sym.st_size );
                    printf( "%04" PRIx32 " ", sym.st_shndx );
                    printf( "%s ", elf_sym_bind_name[sym.get_bind_raw()].name );
                    printf( "%s ", elf_sym_type_name[sym.get_type_raw()].name );
                    printf( "%s\n", str_table.begin() + sym.st_name );
                }
                hlp_print_line();
            }
        }
    }
    
}

void hlprel_print_header() {
    hlp_print_line();
    printf( " offset     addend     sym  type symbol-name\n" );
    hlp_print_line();
}

void ElfFile::print_relocations() {
    if ( is_64bit ) {
        for ( auto i : urange( sh64.size() ) ) {
            auto &sh = sh64[i];
            auto type = sh.get_type();
            if ( type == ElfSecType::SHT_RELA ) {
                printf( "\n[Section %03d]", i );
                
                //Relocation table
                auto rela = get_section_as_table<Elf64_Rela>( sh );
                
                //Corresponding symbol table
                auto &sym_sh = sh64[sh.sh_link];
                auto sym = get_section_as_table<Elf64_Symbol>( sym_sh );
                //Symbol table string table
                auto sym_str = get_section_as_table<char>( sh64[sym_sh.sh_link] );
                
                //Target section
                auto &target = sh64[sh.sh_info];
                printf( " %d relocations", rela.size() );
                printf( " link (sym_table): %03x", sh.sh_link );
                printf( " info (target): %03x\n", sh.sh_info );
                hlprel_print_header();
                for ( auto j : urange( rela.size() ) ) {
                    auto &r = rela[j];
                    printf( " 0x%08" PRIx64 " ", r.r_offset );
                    printf( "0x%08" PRIx64 " ", r.r_addend );
                    printf( "%4" PRIx32 " ", r.get_sym_raw() );
                    printf( "%04" PRIx32 " ", r.get_type_raw() );
                    auto &s = sym[r.get_sym_raw()];
                    printf( "%s", sym_str.begin() + s.st_name );
                    printf( "\n" );
                }
                hlp_print_line();
            }
            else if ( type == ElfSecType::SHT_REL )
                printf( "TODO SHT_REL 64\n" );
        }
    }
    else
        printf( "TODO REL / RELA 32\n" );
        
}


void ElfIdent::print() {
    printf( "Storage class\t= " );
    switch ( get_class() ) {
        case ElfClass::CLASS_32:
            printf( "32-bit objects\n" );
            break;
        case ElfClass::CLASS_64:
            printf( "64-bit objects\n" );
            break;
        default:
            printf( "INVALID CLASS\n" );
            break;
    }
    printf( "Data format\t= " );
    switch ( get_endianness() ) {
        case ElfData::DATA_LSB:
            printf( "2's complement, little endian\n" );
            break;
        case ElfData::DATA_MSB:
            printf( "2's complement, big endian\n" );
            break;
        default:
            printf( "INVALID Format\n" );
            break;
    }
    
    /* OS ABI */
    printf( "OS ABI\t\t= " );
    switch ( get_abi() ) {
        case ElfAbi::ELFOSABI_SYSV:
            printf( "NONE or UNIX System V ABI\n" );
            break;
        case ElfAbi::ELFOSABI_HPUX:
            printf( "HP-UX\n" );
            break;
            
        case ElfAbi::ELFOSABI_NETBSD:
            printf( "NetBSD\n" );
            break;
            
        case ElfAbi::ELFOSABI_LINUX:
            printf( "Linux\n" );
            break;
            
        case ElfAbi::ELFOSABI_SOLARIS:
            printf( "Sun Solaris\n" );
            break;
            
        case ElfAbi::ELFOSABI_AIX:
            printf( "IBM AIX\n" );
            break;
            
        case ElfAbi::ELFOSABI_IRIX:
            printf( "SGI Irix\n" );
            break;
            
        case ElfAbi::ELFOSABI_FREEBSD:
            printf( "FreeBSD\n" );
            break;
            
        case ElfAbi::ELFOSABI_TRU64:
            printf( "Compaq TRU64 UNIX\n" );
            break;
            
        case ElfAbi::ELFOSABI_MODESTO:
            printf( "Novell Modesto\n" );
            break;
            
        case ElfAbi::ELFOSABI_OPENBSD:
            printf( "OpenBSD\n" );
            break;
            
        /*case ELFOSABI_ARM_AEABI:
        printf( "ARM EABI\n" );
        break;*/
        
        case ElfAbi::ELFOSABI_ARM:
            printf( "ARM\n" );
            break;
            
        case ElfAbi::ELFOSABI_STANDALONE:
            printf( "Standalone (embedded) app\n" );
            break;
            
        default:
            printf( "Unknown (0x%x)\n", os_abi );
            break;
    }
}

void Elf32_Header::print() {
    e_ident.print();
    /* ELF filetype */
    printf( "Filetype \t= " );
    switch ( get_file_type() ) {
        case ElfFileType::ET_NONE:
            printf( "N/A (0x0)\n" );
            break;
        case ElfFileType::ET_REL:
            printf( "Relocatable\n" );
            break;
        case ElfFileType::ET_EXEC:
            printf( "Executable\n" );
            break;
        case ElfFileType::ET_DYN:
            printf( "Shared Object\n" );
            break;
        default:
            printf( "Unknown (0x%x)\n", e_type );
            break;
    }
    
    /* ELF Machine-id */
    printf( "Machine\t\t= " );
    switch ( get_machine_type() ) {
        case ElfMachineType::EM_NONE:
            printf( "None (0x0)\n" );
            break;
        case ElfMachineType::EM_386:
            printf( "INTEL x86\n" );
            break;
        case ElfMachineType::EM_X86_64:
            printf( "AMD x86_64\n" );
            break;
        case ElfMachineType::EM_AARCH64:
            printf( "AARCH64\n" );
            break;
        default:
            printf( " 0x%x\n", e_machine );
            break;
    }
    
    /* Entry point */
    printf( "Entry point\t= 0x%08x\n", e_entry );
    
    /* ELF header size in bytes */
    printf( "ELF header size\t= 0x%08x\n", e_ehsize );
    
    /* Program Header */
    printf( "\nProgram Header\t= " );
    printf( "0x%08x\n", e_phoff ); /* start */
    printf( "\t\t  %d entries\n", e_phnum ); /* num entry */
    printf( "\t\t  %d bytes\n", e_phentsize ); /* size/entry */
    
    /* Section header starts at */
    printf( "\nSection Header\t= " );
    printf( "0x%08x\n", e_shoff ); /* start */
    printf( "\t\t  %d entries\n", e_shnum ); /* num entry */
    printf( "\t\t  %d bytes\n", e_shentsize ); /* size/entry */
    printf( "\t\t  0x%08x (string table offset)\n", e_shstrndx );
    
    /* File flags (Machine specific)*/
    printf( "\nFile flags \t= 0x%08x\n", e_flags );
}

void Elf64_Header::print() {
    e_ident.print();
    /* ELF filetype */
    printf( "Filetype \t= " );
    switch ( get_file_type() ) {
        case ElfFileType::ET_NONE:
            printf( "N/A (0x0)\n" );
            break;
        case ElfFileType::ET_REL:
            printf( "Relocatable\n" );
            break;
        case ElfFileType::ET_EXEC:
            printf( "Executable\n" );
            break;
        case ElfFileType::ET_DYN:
            printf( "Shared Object\n" );
            break;
        default:
            printf( "Unknown (0x%x)\n", e_type );
            break;
    }
    
    /* ELF Machine-id */
    printf( "Machine\t\t= " );
    switch ( get_machine_type() ) {
        case ElfMachineType::EM_NONE:
            printf( "None (0x0)\n" );
            break;
        case ElfMachineType::EM_386:
            printf( "INTEL x86\n" );
            break;
        case ElfMachineType::EM_X86_64:
            printf( "AMD x86_64\n" );
            break;
        case ElfMachineType::EM_AARCH64:
            printf( "AARCH64\n" );
            break;
        default:
            printf( " 0x%x\n", e_machine );
            break;
    }
    
    /* Entry point */
    printf( "Entry point\t= 0x%08" PRIx64 "\n", e_entry );
    
    /* ELF header size in bytes */
    printf( "ELF header size\t= 0x%08x\n", e_ehsize );
    
    /* Program Header */
    printf( "\nProgram Header\t= " );
    printf( "0x%08" PRIx64 "\n", e_phoff );  /* start */
    printf( "\t\t  %d entries\n", e_phnum ); /* num entry */
    printf( "\t\t  %d bytes\n", e_phentsize ); /* size/entry */
    
    /* Section header starts at */
    printf( "\nSection Header\t= " );
    printf( "0x%08" PRIx64 "\n", e_shoff );  /* start */
    printf( "\t\t  %d entries\n", e_shnum ); /* num entry */
    printf( "\t\t  %d bytes\n", e_shentsize ); /* size/entry */
    printf( "\t\t  0x%08x (string table offset)\n", e_shstrndx );
    
    /* File flags (Machine specific)*/
    printf( "\nFile flags \t= 0x%08x\n", e_flags );
}

void Elf32_SectionHeader::print( vector_slice<char> &sec_name_table ) {
    printf( "0x%08x ", sh_offset );
    printf( "0x%08x ", sh_addr );
    printf( "0x%08x ", sh_size );
    printf( "%4d ", sh_addralign );
    printf( "0x%08x ", sh_flags );
    bool need = true;
    for ( auto &vn : elf_sec_type_name ) {
        if ( vn.value == sh_type ) {
            printf( "%s ", vn.name );
            need = false;
            break;
        }
    }
    if ( need )
        printf( "%4" PRIx32 " ", sh_type );
    printf( "%s\t", ( sec_name_table.begin() + sh_name ) );
    printf( "\n" );
}

void Elf64_SectionHeader::print( vector_slice<char> &sec_name_table ) {
    printf( "0x%08" PRIx64 " ", sh_offset );
    printf( "0x%08" PRIx64 " ", sh_addr );
    printf( "0x%08" PRIx64 " ", sh_size );
    printf( "%4" PRId64 " ", sh_addralign );
    printf( "0x%08" PRIx64 " ", sh_flags );
    bool need = true;
    for ( auto &vn : elf_sec_type_name ) {
        if ( vn.value == sh_type ) {
            printf( "%s ", vn.name );
            need = false;
            break;
        }
    }
    if ( need )
        printf( "%4" PRIx32 " ", sh_type );
    printf( "%s\t", ( sec_name_table.begin() + sh_name ) );
    printf( "\n" );
}

void Elf32_ProgramHeader::print() {
    printf( "0x%08" PRIx32 " ", p_vaddr );
    printf( "0x%08" PRIx32 " ", p_memsz );
    printf( "0x%08" PRIx32 " ", p_offset );
    printf( "0x%08" PRIx32 " ", p_filesz );
    printf( "%8" PRId32 " ", p_align );
    printf( "0x%08" PRIx32 " ", p_flags );
    bool need = true;
    for ( auto &vn : elf_seg_type_name ) {
        if ( vn.value == p_type ) {
            printf( "%s ", vn.name );
            need = false;
            break;
        }
    }
    if ( need )
        printf( "%8" PRIx32 " ", p_type );
    printf( "\n" );
}

void Elf64_ProgramHeader::print() {
    printf( "0x%08" PRIx64 " ", p_vaddr );
    printf( "0x%08" PRIx64 " ", p_memsz );
    printf( "0x%08" PRIx64 " ", p_offset );
    printf( "0x%08" PRIx64 " ", p_filesz );
    printf( "%8" PRId64 " ", p_align );
    printf( "0x%08" PRIx32 " ", p_flags );
    bool need = true;
    for ( auto &vn : elf_seg_type_name ) {
        if ( vn.value == p_type ) {
            printf( "%s ", vn.name );
            need = false;
            break;
        }
    }
    if ( need )
        printf( "%8" PRIx32 " ", p_type );
        
    printf( "\n" );
}
