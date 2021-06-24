/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include "utility/utility.h"
#include "computer/computer_layout.h"
#include <map>

enum class MemAccess {
    NONE,
    READ,
    WRITE,
    FETCH
};

enum class MemAccessError {
    NONE,
    PROT, //The section does not allow the given memory access
    MAPPED //The address is not mapped to a section
};

/*
    Describes a start address and size of a range in memory.
    get_local_index() returns the index into the range as if it was an array for a given global address.
*/
struct MemoryRange {
    ulong start_address;
    uint size;
    inline bool contains( ulong address ) const {
        return address >= start_address && address < start_address + ( ulong ) size;
    }
    
    inline uint get_local_index( ulong address ) {
        throw_assert( contains( address ), "get_local_index on address not in memory range." );
        return ( uint )( address - start_address );
    }
    
    MemoryRange() : start_address( 0 ), size( 0 ) {}
    MemoryRange( ulong start_address, uint size ) : start_address( start_address ), size( size ) {}
    
    //Used for range sorting
    bool operator<( const MemoryRange &other ) const {
        if (size == 0) {
            if (other.contains(start_address)) return false;
        }
        else if (other.size == 0) {
            if (contains(other.start_address)) return false;
        }
        return start_address < other.start_address;
    }
};


/*
    Annotations are used to mark memory addresses or ranges.
    An Annotation has a name, a type and an optional param (ulong).
    The param is used for example by the syscall system to give the index into the SysCall
    array storing name and procedure of system function implementations.

    The HANDLE type is used by the Handles section.
    The FUNC (function) type is used to mark function symbols (addresses).
    The OBJECT type is used to mark object symbols from the system or the loaded program.

    Annotations can be easily search for by address (from the section) or by type and name
    from the Annotations structure (which holds all the annotations).
*/
struct Annotation {
    ulong base;
    ulong param;
    std::string name;
    enum class Type {
        NONE,
        HANDLE,
        FUNC,
        OBJECT,
    } type;
    Annotation() : base(0), param(0), type(Type::NONE) {}
    Annotation( std::string const &name, Type type, ulong param = 0 )
        : base( 0 ), name( name ), type( type ), param( param ) {}
};

/*
    The AnnotationTable structure is used by SectionAnnotations to store annotations.
*/
struct AnnotationTable {

    static constexpr ulong DEFAULT_ANNOTATION_SIZE = 128;
	std::vector<Annotation> annotations;
    //'Stack' position in the annotations array (= first free position).
    uint annotation_pos = 0;
    
    void init();
    
    //Return the id into the table of the new annotation
    uint new_annotation( ulong base, Annotation const &annotation );
};

/*
    A SectionAnnotations is used in combination with a MemorySection in order to annotated it.
    An Annotation can be added for an address or an address range.
    get_annotation() returns nullptr when no annotation exists at the given address.
*/
struct SectionAnnotations {
    AnnotationTable *annotation_table;
    MemoryRange address_range;
	std::vector<uint> annotation_id;
    
    SectionAnnotations() : annotation_table( nullptr ) {}
    
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
    link() the section before using any method (performed by the Memory when allocating the section).
    Use init() to declare the section in virtual memory with given execute/read/write rights.
    Use upload() to write the data of the section to the engine memory. (Starting at section start)
    Use set_mapped_range() to specify that the content of the section comes from a loaded file.

    The Annotations are not enabled by default. Use init_annotations() to do so.
    print_annotation() will output annotation information about an address (if the address has an annotation)
    print_address_info() will print information about the section before printing the information from print_annotation()
*/
struct MemorySection {
    void *internal_uc;
    Memory *mem;
    std::string mod;
    std::string name;
    MemoryRange address_range;
    MemoryRange mapped_range;
    ulong file_pos;
    ulong page_size;
    bool p_execute;
    bool p_read;
    bool p_write;
    
    SectionAnnotations annotations;
    
    MemorySection() : internal_uc( nullptr ), mem( nullptr ), file_pos(0), p_read(false), p_execute(false), p_write(false), page_size(0) {}
    
    bool init( MemoryRange address_range, std::string const &name, std::string const &mod,
               bool execute, bool read, bool write );
    bool upload( MemoryRange range, char *data );
    
    void link( void *uc, ulong page_size, Memory &mem );
    void set_mapped_range( MemoryRange mapped_range, ulong file_pos ) {
        this->mapped_range = mapped_range;
        this->file_pos = file_pos;
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
    
    void print_address_info( ulong virtual_address, char* buffer, int buffer_size);
    void print_annotation( ulong virtual_address, char* buffer, int buffer_size);
    
};



/*
    A Simple allocation stack structure on top of a MemorySection.
    get_annotated(size) allocates 'size' bytes on top of the stack.
    get_annotated_8byte() allocates a 8 byte slot on top of the stack.
*/
struct SectionStack {
        uint pos;
        MemorySection *mem;
        
        SectionStack() : mem( nullptr ), pos( 0 ) {}
        void init( MemorySection *mem );
        
        bool loaded() {
            return mem != nullptr;
        }
        
        MemoryRange get_annotated( uint size, const std::string &name, Annotation::Type type = Annotation::Type::NONE );
        uint64_t get_annotated_8byte( const std::string &name, Annotation::Type type = Annotation::Type::NONE, ulong param = 0 );
        
    private:
        MemoryRange get_range( uint size );
        uint64_t get_8byte_slot();
};




/*
    The Memory structure handles the creation of MemorySections, writing/reading memory,
    has helper read/write methods for strings and helper function for printing virtual
    addresses info for debugging.

    MemorySections are created by the VirtualHeap, VirtualStack, SystemCalls, Handles and
    by the DLL/ELF loader for each library section.

    The read methods return the data in a unique buffer, so subsequent calls to any read method
    invalidates the data from the previous call.
*/
struct Memory {
    static constexpr ulong BUFFER_START_SIZE = 4096;
    static constexpr ulong MAX_BUFFER_SIZE = 1048576;
    void *internal_uc;
    ulong page_size;
    
    AnnotationTable annotation_table;
	std::vector<std::unique_ptr<MemorySection>> sections;
    uint section_count;
    std::map<MemoryRange, uint> section_lookup; //Sorts the section ids by their addresses
    
	std::vector<uint8_t> buffer;
    MemorySection *sys_section;
    SectionStack sys_section_stack;

    MemorySection *exchange_section;
    
    Memory() : internal_uc( nullptr ), section_count( 0 ), page_size(0), sys_section(nullptr) {}
    
    void init( void *uc );
    bool loaded() {
        return internal_uc != nullptr;
    }
    
    void *read_memory( ulong address, ulong size );
    void write_memory( ulong address, ulong size, void *data );
    
    inline void* read_memory(MemoryRange range) {
        return read_memory(range.start_address, range.size);
    }
    inline void write_memory(MemoryRange range, void* data) {
        write_memory(range.start_address, range.size, data);
    }


    void write_memory_buffer(ulong address, ulong size);
    inline void write_memory_buffer(MemoryRange range) {
        write_memory_buffer(range.start_address, range.size);
    }
    
    MemorySection &new_section( MemoryRange range, const std::string &name, const std::string &module, bool exec, bool read,
                                bool write );
    MemorySection *get_section( ulong virtual_address );
    
    void print_address_info( ulong virtual_address, char* buffer, int buffer_size);
    void print_annotation( ulong virtual_address, char* buffer, int buffer_size);
    
    wchar_t *read_wstr( ulong address );
    char *read_wstr_as_str( ulong address );
    char *read_str( ulong address );
    
    void write_wstr( ulong address, std::string const &text );
    void write_str( ulong address, std::string const &text );
    
    void write_long_word( ulong address, ulong value );
    
    
    static MemAccess get_mem_access( uint type );
    static MemAccessError get_mem_err( uint type );
};

/*
    The Handle structure creates a section in virtual memory where emulated system functions can allocate
    dummy handles to give back to the calling code (the virtual address of these dummies is returned, allowing to
    catch a potential memory access in the handle section).
*/
struct Handles {
    MemorySection *section;
    SectionStack handle_stack;
    
    Handles() : section( nullptr ) {}
    
    void init( Memory &mem );
    
    bool loaded() {
        return section != nullptr;
    }
    
    ulong add_handle( const char *name );
};

/*
    VirtualHeap is a simple implementation of a heap used by the alloc/free system call functions.
    It allocates in block of BLOCK_SIZE bytes.
*/
struct VirtualHeap {
    MemorySection *section;
    ulong heap_handle;
    ulong heap_size;
	vector_bool free_map;
	std::vector<uint> size_map;
    static constexpr ulong BLOCK_SIZE = 0x100;
    static constexpr ulong HEAP_BLOCKS = ComputerLayout::HEAP_SIZE / BLOCK_SIZE;
    
    VirtualHeap() : section( nullptr ), heap_handle(0), heap_size(0) {}
    
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
    
    VirtualStack() : section( nullptr ), registers( nullptr ), stack_start(0), stack_size(0) {}
    
    bool loaded() {
        return section != nullptr && registers != nullptr;
    }
    
    void init( Memory &mem, Registers &regs );
    
    ulong pop_long();
    void push_long( ulong val );
    uint pop_int();
    void push_int( uint val );
};
