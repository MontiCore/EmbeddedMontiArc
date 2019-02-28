#pragma once
#include <stdint.h>
#include "utility.h"

struct ValueName {
    uint32_t value;
    const char *name;
    ValueName() : value( 0 ), name( "" ) {}
    template<typename T>
    ValueName( T value, const char *name ) : value( static_cast<uint32_t>( value ) ), name( name ) {}
};

/* 32-bit ELF base types. */
typedef uint32_t    Elf32_Addr;
typedef uint16_t    Elf32_Half;
typedef uint32_t    Elf32_Off;
typedef int32_t     Elf32_Sword;
typedef uint32_t    Elf32_Word;

/* 64-bit ELF base types. */
typedef uint64_t    Elf64_Addr;
typedef uint16_t    Elf64_Half;
typedef int16_t     Elf64_SHalf;
typedef uint64_t    Elf64_Off;
typedef int32_t     Elf64_Sword;
typedef uint32_t    Elf64_Word;
typedef uint64_t    Elf64_Xword;
typedef int64_t     Elf64_Sxword;


/*

    IDENT

*/


struct ElfMag {
    static constexpr uchar MAG0 = 0x7f;
    static constexpr uchar MAG1 = 'E';
    static constexpr uchar MAG2 = 'L';
    static constexpr uchar MAG3 = 'F';
    static const char *MAG;
};


enum class ElfClass {
    CLASS_NONE = 0,
    CLASS_32 = 1,
    CLASS_64 = 2,
    CLASS_NUM = 3,
};

using ElfArchitectureSize = ElfClass;

enum class ElfData {
    DATA_NONE = 0, //None
    DATA_LSB = 1, //Little endian
    DATA_MSB = 2 //Big Endian
};

using ElfEndianness = ElfData;


enum class ElfVersion {
    NONE = 0,
    CURRENT = 1
};

enum class ElfAbi {
    ELFOSABI_NONE = 0,
    
    ELFOSABI_SYSV = 0,
    ELFOSABI_HPUX = 1,
    ELFOSABI_NETBSD = 2,
    ELFOSABI_LINUX = 3,
    ELFOSABI_HURD = 4,
    ELFOSABI_86OPEN = 5,
    ELFOSABI_SOLARIS = 6,
    ELFOSABI_MONTEREY = 7,
    ELFOSABI_AIX = 7,
    ELFOSABI_IRIX = 8,
    ELFOSABI_FREEBSD = 9,
    ELFOSABI_TRU64 = 10,
    ELFOSABI_MODESTO = 11,
    ELFOSABI_OPENBSD = 12,
    ELFOSABI_NON_STOP = 13,
    ELFOSABI_OPENVMS = 14,
    ELFOSABI_AROS = 15,
    ELFOSABI_FENIXOS = 16,
    ELFOSABI_CLOUDABI = 17,
    ELFOSABI_ARM = 97,
    ELFOSABI_STANDALONE = 255,
};

enum class ElfFileType {
    ET_NONE = 0,
    ET_REL = 1,
    ET_EXEC = 2,
    ET_DYN = 3,
    ET_CORE = 4,
    ET_LOPROC = 0xff00,
    ET_HIPROC = 0xffff,
};

enum class ElfMachineType {
    EM_NONE = 0,
    EM_M32 = 1,
    EM_SPARC = 2,
    EM_386 = 3,
    EM_68K = 4,
    EM_88K = 5,
    EM_486 = 6,  /* Perhaps disused */
    EM_860 = 7,
    EM_MIPS = 8,  /* MIPS R3000 (officially, big-endian only) */
    /* Next two are historical and binaries and
    modules of these types will be rejected by
    Linux.  */
    EM_MIPS_RS3_LE = 10, /* MIPS R3000 little-endian */
    EM_MIPS_RS4_BE = 10, /* MIPS R4000 big-endian */
    EM_PARISC = 15, /* HPPA */
    EM_SPARC32PLUS = 18, /* Sun's "v8plus" */
    EM_PPC = 20, /* PowerPC */
    EM_PPC64 = 21,  /* PowerPC64 */
    EM_SPU = 23, /* Cell BE SPU */
    EM_ARM = 40, /* ARM 32 bit */
    EM_SH = 42, /* SuperH */
    EM_SPARCV9 = 43, /* SPARC v9 64-bit */
    EM_H8_300 = 46, /* Renesas H8/300 */
    EM_IA_64 = 50, /* HP/Intel IA-64 */
    EM_X86_64 = 62, /* AMD x86-64 */
    EM_S390 = 22, /* IBM S/390 */
    EM_CRIS = 76, /* Axis Communications 32-bit embedded processor */
    EM_M32R = 88, /* Renesas M32R */
    EM_MN10300 = 89, /* Panasonic/MEI MN10300, AM33 */
    EM_OPENRISC = 92,    /* OpenRISC 32-bit embedded processor */
    EM_XTENSA = 94, /* Tensilica Xtensa Architecture */
    EM_BLACKFIN = 106,    /* ADI Blackfin Processor */
    EM_ALTERA_NIOS2 = 113, /* Altera Nios II soft-core processor */
    EM_TI_C6000 = 140, /* TI C6X DSPs */
    EM_AARCH64 = 183, /* ARM 64 bit */
    EM_TILEPRO = 188, /* Tilera TILEPro */
    EM_MICROBLAZE = 189, /* Xilinx MicroBlaze */
    EM_TILEGX = 191, /* Tilera TILE-Gx */
    EM_RISCV = 243, /* RISC-V */
    EM_BPF = 247, /* Linux BPF - in-kernel virtual machine */
    EM_CSKY = 252, /* C-SKY */
    EM_FRV = 0x5441, /* Fujitsu FR-V */
    /*
    * This is an interim value that we will use until the committee comes
    * up with a final number.
    */
    EM_ALPHA = 0x9026,
    /* Bogus old m32r magic number, used by old tools. */
    EM_CYGNUS_M32R = 0x9041,
    /* This is the old interim value for S/390 architecture */
    EM_S390_OLD = 0xA390,
    /* Also Panasonic/MEI MN10300, AM33 */
    EM_CYGNUS_MN10300 = 0xbeef,
};


/*

PROGRAM HEADER

*/

enum class ElfSegType {
    /* These constants are for the segment types stored in the image headers */
    PT_NULL = 0,
    PT_LOAD = 1,
    PT_DYNAMIC = 2,
    PT_INTERP = 3,
    PT_NOTE = 4,
    PT_SHLIB = 5,
    PT_PHDR = 6,
    PT_TLS = 7,              /* Thread local storage segment */
    PT_LOOS = 0x60000000,     /* OS-specific */
    PT_HIOS = 0x6fffffff,     /* OS-specific */
    PT_LOPROC = 0x70000000,
    PT_HIPROC = 0x7fffffff,
    PT_GNU_EH_FRAME = 0x6474e550,
    
    PT_GNU_STACK = ( PT_LOOS + 0x474e551 )
};

enum class Elf32_ProgramPerm {
    PF_R = 0x4,
    PF_W = 0x2,
    PF_X = 0x1,
};

struct Elf32_ProgramHeader {
    Elf32_Word      p_type;
    Elf32_Off       p_offset;
    Elf32_Addr      p_vaddr;
    Elf32_Addr      p_paddr;
    Elf32_Word      p_filesz;
    Elf32_Word      p_memsz;
    Elf32_Word      p_flags;
    Elf32_Word      p_align;
    
    ElfSegType get_type() {
        return static_cast<ElfSegType>( p_type );
    }
    bool has_perm( Elf32_ProgramPerm perm ) {
        return p_flags & static_cast<Elf32_Word>( perm );
    }
};

struct Elf64_ProgramHeader {
    Elf64_Word      p_type;
    Elf64_Word      p_flags;
    Elf64_Off       p_offset;
    Elf64_Addr      p_vaddr;
    Elf64_Addr      p_paddr;
    Elf64_Xword     p_filesz;
    Elf64_Xword     p_memsz;
    Elf64_Xword     p_align;
    
    ElfSegType get_type() {
        return static_cast<ElfSegType>( p_type );
    }
    bool has_perm( Elf32_ProgramPerm perm ) {
        return p_flags & static_cast<Elf64_Word>( perm );
    }
};




/*

    SECTION HEADER

*/

/* special section indexes */
#define SHN_UNDEF   0
#define SHN_LORESERVE   0xff00
#define SHN_LOPROC  0xff00
#define SHN_HIPROC  0xff1f
#define SHN_LIVEPATCH   0xff20
#define SHN_ABS     0xfff1
#define SHN_COMMON  0xfff2
#define SHN_HIRESERVE   0xffff

enum class ElfSecType : uint32_t {
    SHT_NULL = 0,
    SHT_PROGBITS = 1,
    SHT_SYMTAB = 2,
    SHT_STRTAB = 3,
    SHT_RELA = 4,
    SHT_HASH = 5,
    SHT_DYNAMIC = 6,
    SHT_NOTE = 7,
    SHT_NOBITS = 8,
    SHT_REL = 9,
    SHT_SHLIB = 10,
    SHT_DYNSYM = 11,
    SHT_INIT_ARRAY = 0x0E,
    SHT_FINI_ARRAY = 0x0F,
    SHT_PREINIT_ARRAY = 0x10,
    SHT_GROUP = 0x11,
    SHT_SYMTAB_SHNDX = 0x12,
    SHT_NUM = 0x13,
    SHT_LOOS = 0x60000000,
    SHT_LOPROC = 0x70000000,
    SHT_HIPROC = 0x7fffffff,
    SHT_LOUSER = 0x80000000,
    SHT_HIUSER = 0xffffffff,
};

extern ValueName elf_sec_type_name[static_cast<uint>( ElfSecType::SHT_NUM )];

enum class ElfSecFlags : uint32_t {
    SHF_WRITE = 0x1,
    SHF_ALLOC = 0x2,
    SHF_EXECINSTR = 0x4,
    SHF_MERGE = 0x10,
    SHF_STRINGS = 0x20,
    SHF_INFO_LINK = 0x40,
    SHF_LINK_ORDER = 0x80,
    SHF_OS_NONCONFORMING = 0x100,
    SHF_GROUP = 0x200,
    SHF_TLS = 0x400,
    SHF_MASKOS = 0x0ff00000,
    SHF_RELA_LIVEPATCH = 0x00100000,
    SHF_RO_AFTER_INIT = 0x00200000,
    SHF_MASKPROC = 0xf0000000,
};

struct Elf32_SectionHeader {
    Elf32_Word      sh_name;
    Elf32_Word      sh_type;
    Elf32_Word      sh_flags;
    Elf32_Addr      sh_addr;
    Elf32_Off       sh_offset;
    Elf32_Word      sh_size;
    Elf32_Word      sh_link;
    Elf32_Word      sh_info;
    Elf32_Word      sh_addralign;
    Elf32_Word      sh_entsize;
    
    ElfSecType get_type() {
        return static_cast<ElfSecType>( sh_type );
    }
    bool has_flag( ElfSecFlags flag ) {
        return sh_flags & static_cast<Elf32_Word>( flag );
    }
    void print( ArraySlice<char> &sec_name_table );
};

struct Elf64_SectionHeader {
    Elf64_Word      sh_name;
    Elf64_Word      sh_type;
    Elf64_Xword     sh_flags;
    Elf64_Addr      sh_addr;
    Elf64_Off       sh_offset;
    Elf64_Xword     sh_size;
    Elf64_Word      sh_link;
    Elf64_Word      sh_info;
    Elf64_Xword     sh_addralign;
    Elf64_Xword     sh_entsize;
    
    ElfSecType get_type() {
        return static_cast<ElfSecType>( sh_type );
    }
    bool has_flag( ElfSecFlags flag ) {
        return sh_flags & static_cast<Elf64_Xword>( flag );
    }
    void print( ArraySlice<char> &sec_name_table );
};




/*
    ELF HEADER
*/


struct ElfIdent {
    uchar       mag0;
    uchar       mag1;
    uchar       mag2;
    uchar       mag3;
    uchar       clss;
    uchar       data;
    uchar       version;
    uchar       os_abi;
    uchar       abi_version;
    uchar       pad[7];
    
    bool is_elf() {
        return mag0 == ElfMag::MAG0 &&
               mag1 == ElfMag::MAG1 &&
               mag2 == ElfMag::MAG2 &&
               mag3 == ElfMag::MAG3;
    }
    ElfClass get_class() {
        return static_cast<ElfClass>( clss );
    }
    ElfData get_endianness() {
        return static_cast<ElfData>( data );
    }
    ElfVersion get_version() {
        return static_cast<ElfVersion>( version );
    }
    ElfAbi get_abi() {
        return static_cast<ElfAbi>( os_abi );
    }
    void print();
};
static_assert( sizeof( ElfIdent ) == 16 );

struct Elf32_Header {
    ElfIdent        e_ident;
    Elf32_Half      e_type;
    Elf32_Half      e_machine;
    Elf32_Word      e_version;
    Elf32_Addr      e_entry;
    Elf32_Off       e_phoff;
    Elf32_Off       e_shoff;
    Elf32_Word      e_flags;
    Elf32_Half      e_ehsize;
    Elf32_Half      e_phentsize;
    Elf32_Half      e_phnum;
    Elf32_Half      e_shentsize;
    Elf32_Half      e_shnum;
    Elf32_Half      e_shstrndx;
    bool section_header_size_valid() {
        return e_shentsize == sizeof( Elf32_SectionHeader );
    }
    bool program_header_size_valid() {
        return e_phentsize == sizeof( Elf32_ProgramHeader );
    }
    ElfFileType get_file_type() {
        return static_cast<ElfFileType> ( e_type );
    }
    ElfMachineType get_machine_type() {
        return static_cast<ElfMachineType> ( e_machine );
    }
    void print();
};
static_assert( sizeof( Elf32_Header ) == 52 );


struct Elf64_Header {
    ElfIdent        e_ident;
    Elf64_Half      e_type;
    Elf64_Half      e_machine;
    Elf64_Word      e_version;
    Elf64_Addr      e_entry;
    Elf64_Off       e_phoff;
    Elf64_Off       e_shoff;
    Elf64_Word      e_flags;
    Elf64_Half      e_ehsize;
    Elf64_Half      e_phentsize;
    Elf64_Half      e_phnum;
    Elf64_Half      e_shentsize;
    Elf64_Half      e_shnum;
    Elf64_Half      e_shstrndx;
    
    bool section_header_size_valid() {
        return e_shentsize == sizeof( Elf64_SectionHeader );
    }
    bool program_header_size_valid() {
        return e_phentsize == sizeof( Elf64_ProgramHeader );
    }
    ElfFileType get_file_type() {
        return static_cast<ElfFileType> ( e_type );
    }
    ElfMachineType get_machine_type() {
        return static_cast<ElfMachineType> ( e_machine );
    }
    void print();
};
static_assert( sizeof( Elf64_Header ) == 64 );



/*

    SYMBOL TABLE

*/

enum class ElfSymbolBind {
    STB_LOCAL  = 0,
    STB_GLOBAL = 1,
    STB_WEAK   = 2,
};
enum class ElfSymbolType {
    STT_NOTYPE  = 0,
    STT_OBJECT  = 1,
    STT_FUNC    = 2,
    STT_SECTION = 3,
    STT_FILE    = 4,
    STT_COMMON  = 5,
    STT_TLS     = 6,
};

struct Elf32_Symbol {
    Elf32_Word    st_name;
    Elf32_Addr    st_value;
    Elf32_Word    st_size;
    unsigned char st_info;
    unsigned char st_other;
    Elf32_Half    st_shndx;
    
    ElfSymbolBind get_bind() {
        return static_cast<ElfSymbolBind>( ( st_info ) >> 4 );
    }
    ElfSymbolType get_type() {
        return static_cast<ElfSymbolType>( ( ( unsigned int )st_info ) & 0xf );
    }
};

struct Elf64_Symbol {
    Elf64_Word st_name;       /* Symbol name, index in string tbl */
    unsigned char st_info;    /* Type and binding attributes */
    unsigned char st_other;   /* No defined meaning, 0 */
    Elf64_Half st_shndx;      /* Associated section index */
    Elf64_Addr st_value;      /* Value of the symbol */
    Elf64_Xword st_size;      /* Associated symbol size */
    
    ElfSymbolBind get_bind() {
        return static_cast<ElfSymbolBind>( ( st_info ) >> 4 );
    }
    ElfSymbolType get_type() {
        return static_cast<ElfSymbolType>( ( ( unsigned int )st_info ) & 0xf );
    }
};





















struct ElfFile {
    Array<uchar> data;
    ElfIdent *ident_ptr;
    bool is_64bit;
    Elf32_Header *header32_ptr;
    Elf64_Header *header64_ptr;
    ArraySlice<Elf32_SectionHeader> sh32;
    ArraySlice<Elf64_SectionHeader> sh64;
    ArraySlice<Elf32_ProgramHeader> ph32;
    ArraySlice<Elf64_ProgramHeader> ph64;
    //ArraySlice<Elf32_Symbol> sym32;
    //ArraySlice<Elf64_Symbol> sym64;
    //ArraySlice<char> str_table;
    ArraySlice<char> sec_name_table;
    
    bool parse();
    
    
    ElfIdent &ident() {
        return *ident_ptr;
    }
    Elf32_Header &header32() {
        return *header32_ptr;
    }
    Elf64_Header &header64() {
        return *header64_ptr;
    }
    
    
    void print();
    
    void print_sections();
    void print_segments();
    void print_symbols();
};



