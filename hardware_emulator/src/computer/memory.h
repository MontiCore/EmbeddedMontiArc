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

/*
    Describes a start address and size of a range in memory.
    get_local_index() returns the index into the range as if it was an array for a given global address.
*/
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


/*
    Annotations are used to mark memory addresses or ranges.
    An Annotation has a name, a type and an optional param (ulong).
    The param is used for instance by the syscall system to give the index into the SysCall
    array storing name and procedure of syscall implementations.

    The HANDLE type is used by the Handles section.
    The PROC (procedure) type is used by the syscall annotations.
    The SYMBOL type is used to annotate System symbols and DLL symbols.
    The LIBRARY_EXPORT type is used by the code loader to annotate (and thus list) exported
    library functions (entry points for the different functions of a DLL/Archive)

    Annotations can be easily search for by address (from the section) or by type and name
    from the Annotations structure (which holds old the annotations).
*/
struct Annotation {
    enum Type {
        NONE,
        HANDLE,
        FUNC,
        OBJECT,
    };
    ulong base;
    std::string name;
    Type type;
    ulong param;
    Annotation() {}
    Annotation( std::string const &name, Type type, ulong param = 0 )
        : base( 0 ), name( name ), type( type ), param( param ) {}
};

/*
    The AnnotationCollection structure is used by SectionAnnotation to store annotations
    and is used to look through all annotations. (But it uses linear search).
*/
struct AnnotationTable {

    static constexpr ulong DEFAULT_ANNOTATION_SIZE = 128;
    Array<Annotation> annotations;
    //'Stack' position in the annotations array (= first free position).
    uint annotation_pos;
    
    void init();
    
    uint new_annotation( ulong base, Annotation const &annotation );
};

/*
    A SectionAnnotation is to be used in combination with a MemorySection in order to annotated it.
    An Annotation can be added for a address or an address range.
    get_annotation() returns nullptr when no annotation exists at the given address.
*/
struct SectionAnnotation {
    AnnotationTable *annotation_table;
    MemoryRange address_range;
    Array<uint> annotation_id;
    
    SectionAnnotation() : annotation_table( nullptr ) {}
    
    void init( AnnotationTable *annotation_table, MemoryRange address_range );
    
    void add_annotation( MemoryRange range, Annotation const &annotation );
    void add_annotation( ulong address, Annotation const &annotation );
    Annotation *get_annotation( ulong address );
    
    bool loaded() {
        return annotation_table != nullptr;
    }
};

struct Memory;

/*
    A MemorySection represents a Virtual memory section in the Unicorn engine.
    link() the section before using any method.
    Use init() to declare the section in virtual memory with given execute/read/write rights.
    Use upload() to write the data of the section to the engine memory. (Starting at section start)

    Use write() to write data to a custom location within the section.
    Use read() to write data from a custom location within the section.
    read() uses the Memory::read() function, which stores the result in a unique buffer, meaning subsequent
    calls to read() invalidate the data from previous calls.

    To get numbers from the byte stream use the Utility (namespace) functions.
*/
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
    bool upload( ulong address, char *data, uint64_t size );
    
    void *read( ulong address, ulong size );
    void write( ulong address, ulong size, void *data );
    
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
    
    void print_address_info( ulong virtual_address );
    void print_annotation( ulong virtual_address );
};



/*
    A Simple allocating Stack structure on top of a MemorySection.
    get_range(size) allocates 'size' bytes on top of the stack.
    get_8byte_slot() allocates a 8 byte slot on top of the stack.
*/
struct SectionStack {
        uint pos;
        MemorySection *mem;
        
        SectionStack() : mem( nullptr ), pos( 0 ) {}
        void init( MemorySection *mem );
        
        bool loaded() {
            return mem != nullptr;
        }
        
        MemoryRange get_annotated( uint size, const std::string &name, Annotation::Type type = Annotation::NONE );
        uint64_t get_annotated_8byte( const std::string &name, Annotation::Type type = Annotation::NONE, ulong param = 0 );
        
    private:
        MemoryRange get_range( uint size );
        uint64_t get_8byte_slot();
};




/*
    The Memory structure handles the creation of MemorySections, writing/reading memory,
    helper read/write methods for strings and printing virtual addresses info for debuggin.

    MemorySections are created by the VirtualHeap, VirtualStack, SystemCalls, Handles and
    by the DLL/Archive loader for each library section.

    The read methods return the data in a unique buffer, so subsequent calls to any read method
    invalidates the data from the previous call.
*/
struct Memory {
    static constexpr ulong BUFFER_SIZE = 4096;
    static constexpr ulong SECTION_SIZE = 128;
    void *internal_uc;
    ulong page_size;
    
    AnnotationTable annotation_table;
    Array<MemorySection> sections;
    uint section_pos;
    
    Array<uint8_t> buffer;
    
    MemorySection *sys_section;
    SectionStack sys_section_stack;
    
    Memory() : internal_uc( nullptr ), section_pos( 0 ) {}
    
    void init( void *uc );
    bool loaded() {
        return internal_uc != nullptr;
    }
    
    void *read_memory( ulong address, ulong size );
    void write_memory( ulong address, ulong size, void *data );
    
    void *read_memory( MemoryRange range );
    void write_memory( MemoryRange range, void *data );
    
    MemorySection &new_section();
    MemorySection *get_section( ulong virtual_address );
    
    void print_address_info( ulong virtual_address );
    void print_annotation( ulong virtual_address );
    
    wchar_t *read_wstr( ulong address );
    uchar *read_wstr_as_str( ulong address );
    uchar *read_str( ulong address );
    
    void write_wstr( ulong address, std::string const &text );
    void write_str( ulong address, std::string const &text );
    
    void write_long_word( ulong address, ulong value );
};

/*
    The Handle structure creates a section in virtual memory where emulated system functions can allocate
    dummy handles to give back to the calling code (the *address* of these dummies is returned, allowing to
    catch a potential memory access is the handle section).
*/
struct Handles {
    MemorySection *section;
    SectionStack handle_stack;
    
    Handles() : section( nullptr ) {}
    
    void init( Memory &mem );
    
    bool loaded() {
        return section != nullptr;
    }
    
    //ulong get_handle( const char *name );
    ulong add_handle( const char *name );
};

/*
    VirtualHeap is a simple implementation of a heap used by the alloc/free system call functions.
*/
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

/*
    VirtualStack is the isolated virtual memory section reserved for the program stack, with wrapper functions
    allowing to simulate pop/push from outside the emulator (for manual reading/placing of data).
*/
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
    uint pop_int();
    void push_int( uint val );
};
