#pragma once
#include "utility.h"
#include "computer/computer_layout.h"


enum class MemAccess {
    NONE,
    READ,
    WRITE,
    FETCH
};

enum class MemAccessError {
    NONE,
    PROT,
    MAPPED
};

struct MemoryRange {
    ulong start_address;
    uint size;
    inline bool contains( ulong address ) {
        return address >= start_address && address < start_address + ( ulong ) size;
    }
    
    inline uint get_local_index( ulong address ) {
        throw_assert( contains( address ), "get_local_index on address not in memory range." );
        return ( uint )( address - start_address );
    }
    
    MemoryRange() : start_address( 0 ), size( 0 ) {}
    MemoryRange( ulong start_address, uint size ) : start_address( start_address ), size( size ) {}
};



struct Annotation {
    enum Type {
        NONE,
        HANDLE = BIT( 1 ),
        PROC = BIT( 2 ),
        SYMBOL = BIT( 3 ),
        LIBRARY_EXPORT = BIT( 4 )
    };
    ulong base;
    std::string name;
    Type type;
    ulong param;
    Annotation() {}
    Annotation( std::string const &name, Type type, ulong param = 0 )
        : base( 0 ), name( name ), type( type ), param( param ) {}
};

struct Annotations {

    static constexpr ulong DEFAULT_ANNOTATION_SIZE = 128;
    Array<Annotation> annotations;
    uint annotation_pos;
    
    Annotations() : annotations( DEFAULT_ANNOTATION_SIZE ) {}
    
    void init_annotations();
    
    uint new_annotation( ulong base, Annotation const &annotation );
    Annotation &get_annotation( std::string const &name, uint type_mask );
    uint64_t get_handle( std::string const &name, uint type_mask );
};

struct SectionAnnotation {
    Annotations *annotations;
    MemoryRange address_range;
    Array<bool> annotated;
    Array<uint> annotation_id;
    
    SectionAnnotation() : annotations( nullptr ) {}
    
    void init( Annotations *annotations, MemoryRange address_range );
    
    void add_annotation( MemoryRange range, Annotation const &annotation );
    void add_annotation( ulong address, Annotation const &annotation );
    Annotation *get_annotation( ulong address );
    
    bool loaded() {
        return annotations != nullptr;
    }
};

struct Memory;
struct MemorySection {
    void *internal_uc;
    Memory *mem;
    std::string mod;
    std::string name;
    MemoryRange address_range;
    MemoryRange file_range;
    ulong page_size;
    bool p_execute;
    bool p_read;
    bool p_write;
    
    SectionAnnotation annotations;
    
    MemorySection() : internal_uc( nullptr ), mem( nullptr ) {}
    
    bool init( MemoryRange address_range, std::string const &name, std::string const &mod,
               bool execute, bool read, bool write );
    bool upload( char *data, uint64_t size );
    
    uchar *read( ulong address, ulong size );
    void write( ulong address, ulong size, uchar *data );
    
    void link( void *uc, ulong page_size, Memory &mem );
    void set_file_range( MemoryRange file_range ) {
        this->file_range = file_range;
    }
    
    uint64_t address_to_file( uint64_t virtual_address );
    
    bool linked() {
        return internal_uc != nullptr;
    }
    bool loaded() {
        return address_range.size > 0;
    }
    
    bool has_annotations() {
        return annotations.loaded();
    }
    
    void init_annotations();
};




struct SectionStack {
    uint pos;
    MemorySection *mem;
    
    SectionStack() : mem( nullptr ), pos( 0 ) {}
    void init( MemorySection *mem );
    
    bool loaded() {
        return mem != nullptr;
    }
    
    MemoryRange get_range( uint size );
    uint64_t get_8byte_slot();
};





struct Memory {
    static constexpr ulong BUFFER_SIZE = 512;
    static constexpr ulong SECTION_SIZE = 128;
    void *internal_uc;
    ulong page_size;
    
    Annotations annotations;
    Array<MemorySection> sections;
    uint section_pos;
    
    Array<uint8_t> buffer;
    
    MemorySection *sys_section;
    SectionStack sys_section_stack;
    
    Memory() : internal_uc( nullptr ), section_pos( 0 ), sections( SECTION_SIZE ), buffer( BUFFER_SIZE ) {}
    
    void init( void *uc );
    bool loaded() {
        return internal_uc != nullptr;
    }
    
    uchar *read_memory( ulong address, ulong size );
    void write_memory( ulong address, ulong size, uchar *data );
    
    uchar *read_memory( MemoryRange range );
    void write_memory( MemoryRange range, uchar *data );
    
    MemorySection &new_section();
    MemorySection *get_section( ulong virtual_address );
    
    void print_address_info( ulong virtual_address );
    void print_annotation( ulong virtual_address );
    void print_annotation( MemorySection &sec, ulong virtual_address );
    
    wchar_t *read_wstr( ulong address );
    uchar *read_wstr_as_str( ulong address );
    uchar *read_str( ulong address );
    
    void write_wstr( ulong address, std::string const &text );
    void write_str( ulong address, std::string const &text );
    
    void write_long_word( ulong address, ulong value );
};

struct Handles {
    MemorySection *section;
    SectionStack handle_stack;
    
    Handles() : section( nullptr ) {}
    
    void init( Memory &mem );
    
    bool loaded() {
        return section != nullptr;
    }
    
    ulong get_handle( const char *name );
    ulong add_handle( const char *name );
};

struct VirtualHeap {
    MemorySection *section;
    ulong heap_handle;
    ulong heap_size;
    Array<bool> free_map;
    Array<uint> size_map;
    static constexpr ulong BLOCK_SIZE = 0x100;
    static constexpr ulong HEAP_BLOCKS = ComputerLayout::HEAP_SIZE / BLOCK_SIZE;
    
    VirtualHeap() : section( nullptr ) {}
    
    bool loaded() {
        return section != nullptr;
    }
    
    void init( Memory &mem, Handles &handles );
    bool alloc( ulong size, ulong &address );
    bool free( ulong &address );
};

struct Registers;

struct VirtualStack {
    MemorySection *section;
    Registers *registers;
    ulong stack_start;
    ulong stack_size;
    
    VirtualStack() : section( nullptr ), registers( nullptr ) {}
    
    bool loaded() {
        return section != nullptr && registers != nullptr;
    }
    
    void init( Memory &mem, Registers &regs );
    
    ulong pop_long();
    void push_long( ulong val );
};