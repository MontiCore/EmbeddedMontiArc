#include "computer/memory.h"
#include <unicorn/unicorn.h>
#include "computer/computer_layout.h"
#include "computer/registers.h"

#include <iostream>

void AnnotationCollection::init() {
    annotations.init( DEFAULT_ANNOTATION_SIZE );
    annotation_pos = 0;
    new_annotation( 0, Annotation( "NO-NOTE", Annotation::NONE ) );
}

uint AnnotationCollection::new_annotation( ulong base, Annotation const &annotation ) {
    if ( annotation_pos >= annotations.size() )
        annotations.resize( ( annotations.size() ) * 3 / 2 + 1 );
    auto &target = annotations[annotation_pos];
    target = annotation;
    target.base = base;
    return annotation_pos++;
}
Annotation &AnnotationCollection::get_annotation( std::string const &name, uint type_mask ) {
    for ( uint i : urange( annotation_pos ) ) {
        auto &note = annotations[i];
        if ( note.type & type_mask )
            if ( note.name == name )
                return note;
    }
    return annotations[0];
}
uint64_t AnnotationCollection::get_handle( std::string const &name, uint type_mask ) {
    return get_annotation( name, type_mask ).base;
}






void SectionAnnotation::init( AnnotationCollection *annotation_collection, MemoryRange address_range ) {
    this->collection = annotation_collection;
    this->address_range = address_range;
    annotated.init( address_range.size );
    annotation_id.init( address_range.size );
}

void SectionAnnotation::add_annotation( MemoryRange range, Annotation const &annotation ) {
    throw_assert( loaded(), "SectionAnnotation::add_annotation() on uninitialized SectionAnnotation" );
    auto note_id = collection->new_annotation( range.start_address, annotation );
    auto start_index = address_range.get_local_index( range.start_address );
    for ( auto i : urange( start_index, start_index + range.size ) ) {
        annotated[i] = true;
        annotation_id[i] = note_id;
    }
}

void SectionAnnotation::add_annotation( ulong address, Annotation const &annotation ) {
    throw_assert( loaded(), "SectionAnnotation::add_annotation() on uninitialized SectionAnnotation" );
    auto note_id = collection->new_annotation( address, annotation );
    auto index = address_range.get_local_index( address );
    annotated[index] = true;
    annotation_id[index] = note_id;
}

Annotation *SectionAnnotation::get_annotation( ulong address ) {
    throw_assert( loaded(), "SectionAnnotation::get_annotation() on uninitialized SectionAnnotation" );
    auto local_address = address_range.get_local_index( address );
    if ( !annotated[local_address] )
        return nullptr;
    return &( collection->annotations[annotation_id[local_address]] );
}







bool MemorySection::init( MemoryRange address_range, std::string const &name, std::string const &mod,
                          bool execute, bool read, bool write ) {
    throw_assert( linked(), "MemorySection::init() on unlinked MemorySection." );
    this->address_range = address_range;
    this->file_range = address_range;
    this->name = name;
    this->mod = mod;
    this->p_execute = execute;
    this->p_read = read;
    this->p_write = write;
    this->address_range.size = ( ( ( address_range.size - 1 ) / ( uint ) page_size ) + 1 ) * ( uint )page_size;
    uint32_t prot = 0;
    if ( execute )
        prot |= UC_PROT_EXEC;
    if ( read )
        prot |= UC_PROT_READ;
    if ( write )
        prot |= UC_PROT_WRITE;
    if ( uc_mem_map( static_cast<uc_engine *>( internal_uc ), this->address_range.start_address, this->address_range.size,
                     prot ) ) {
        printf( "Error mapping emulator memory!\n" );
        return false;
    }
    return true;
}

bool MemorySection::upload( char *data, uint64_t size ) {
    throw_assert( loaded(), "MemorySection::upload() on uninitialized MemorySection." );
    throw_assert( linked(), "MemorySection::upload() on unlinked MemorySection." );
    throw_assert( size <= address_range.size, "MemorySection::upload() outside section." );
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address_range.start_address, data, size ) ) {
        printf( "Failed to write emulation code to memory, quit!\n" );
        return false;
    }
    return true;
}

uchar *MemorySection::read( ulong address, ulong size ) {
    throw_assert( loaded(), "MemorySection::read() on uninitialized MemorySection." );
    throw_assert( linked(), "MemorySection::read() on unlinked MemorySection." );
    throw_assert( address_range.contains( address ), "MemorySection::read() outside address range." );
    throw_assert( address_range.contains( address + size ), "MemorySection::read() outside address range." );
    return mem->read_memory( address, size );
}

void MemorySection::write( ulong address, ulong size, uchar *data ) {
    throw_assert( loaded(), "MemorySection::write() on uninitialized MemorySection." );
    throw_assert( linked(), "MemorySection::write() on unlinked MemorySection." );
    throw_assert( address_range.contains( address ), "MemorySection::write() outside address range." );
    throw_assert( address_range.contains( address + size ), "MemorySection::write() outside address range." );
    mem->write_memory( address, size, data );
}

void MemorySection::link( void *internal_uc, ulong page_size, Memory &mem ) {
    this->internal_uc = internal_uc;
    this->page_size = page_size;
    this->mem = &mem;
}

uint64_t MemorySection::address_to_file( uint64_t virtual_address ) {
    throw_assert( loaded(), "MemorySection::address_to_file() on uninitialized memory section." );
    if ( file_range.start_address != address_range.start_address && address_range.contains( virtual_address ) ) {
        auto local_pos = address_range.get_local_index( virtual_address );
        if ( local_pos < file_range.size )
            return local_pos + file_range.start_address;
    }
    return virtual_address;
}

void MemorySection::init_annotations() {
    annotations.init( &mem->annotation_collection, address_range );
}





void SectionStack::init( MemorySection *mem ) {
    this->mem = mem;
    pos = 0;
}

MemoryRange SectionStack::get_range( uint size ) {
    throw_assert( loaded(), "SectionStack::get_range() on uninitialized SectionStack" );
    auto temp = pos;
    auto inc = ( ( ( size - 1 ) / 8 ) + 1 ) * 8;
    pos += inc;
    return MemoryRange( temp + mem->address_range.start_address, size );
}
uint64_t SectionStack::get_8byte_slot() {
    throw_assert( loaded(), "SectionStack::get_8byte_slot() on uninitialized SectionStack" );
    auto temp = pos;
    pos += 8;
    return temp + mem->address_range.start_address;
}





void Memory::init( void *uc ) {
    sections.init( SECTION_SIZE );
    buffer.init( BUFFER_SIZE );
    this->internal_uc = uc;
    section_pos = 0;
    uc_query( static_cast<uc_engine *>( internal_uc ), UC_QUERY_PAGE_SIZE, &page_size );
    
    annotation_collection.init();
    
    sys_section = &new_section();
    sys_section->init( MemoryRange( ComputerLayout::SYSPAGE_ADDRESS, ComputerLayout::SYSPAGE_RANGE ), "SYSPAGE", "OS",
                       false, true, true );
    sys_section->init_annotations();
    sys_section_stack.init( sys_section );
}


uint8_t *Memory::read_memory( ulong address, ulong size ) {
    if ( size > BUFFER_SIZE )
        size = BUFFER_SIZE;
    uc_mem_read( static_cast<uc_engine *>( internal_uc ), address, buffer.begin(), size );
    return buffer.begin();
}

void Memory::write_memory( ulong address, ulong size, uchar *data ) {
    auto s = size > BUFFER_SIZE ? BUFFER_SIZE : size;
    for ( uint i : urange( ( uint )s ) )
        buffer[i] = data[i];
    uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buffer.begin(), s );
}

uchar *Memory::read_memory( MemoryRange range ) {
    return read_memory( range.start_address, range.size );
}

void Memory::write_memory( MemoryRange range, uchar *data ) {
    write_memory( range.start_address, range.size, data );
}


MemorySection &Memory::new_section() {
    auto &sec = sections[section_pos++];
    sec.link( internal_uc, page_size, *this );
    return sec;
}

MemorySection *Memory::get_section( ulong virtual_address ) {
    for ( uint i : urange( section_pos ) ) {
        if ( sections[i].address_range.contains( virtual_address ) )
            return sections.begin() + i;
    }
    return nullptr;
}

void MemorySection::print_address_info( ulong virtual_address ) {
    auto file_address = address_to_file( virtual_address );
    Log::info << "[";
    Log::note << mod;
    Log::info << ":";
    Log::note << name;
    if ( file_address != virtual_address )
        Log::white << to_hex( file_address, 16, true );
    Log::info << "] ";
    print_annotation( virtual_address );
}

void MemorySection::print_annotation( ulong virtual_address ) {
    if ( has_annotations() ) {
        auto note_ptr = annotations.get_annotation( virtual_address );
        if ( note_ptr ) {
            auto &note = *note_ptr;
            Log::info << "(";
            Log::note << note.name;
            if ( note.base != virtual_address )
                //Log::info << "[" << to_hex( virtual_address - note.base, 0 ) << "]";
                Log::info << "[" << ( virtual_address - note.base ) << "]";
            Log::info << ") ";
        }
    }
}

void Memory::print_address_info( ulong  virtual_address ) {
    auto sec_ptr = get_section( virtual_address );
    if ( sec_ptr )
        sec_ptr->print_address_info( virtual_address );
    else
        Log::info << "[NON-ALLOCATED] ";
}

void Memory::print_annotation( ulong virtual_address ) {
    auto sec_ptr = get_section( virtual_address );
    if ( sec_ptr )
        sec_ptr->print_annotation( virtual_address );
}



wchar_t *Memory::read_wstr( ulong address ) {
    uint size = 0;
    wchar_t c;
    do {
        uc_mem_read( static_cast<uc_engine *>( internal_uc ), address + size, buffer.begin() + size, 2 );
        c = *( wchar_t * )( buffer.begin() + size );
        size += 2;
    } while ( c != 0 && size < BUFFER_SIZE );
    
    if ( size >= BUFFER_SIZE )
        *( wchar_t * )( buffer.begin() + ( size - 2 ) ) = 0;
    return ( wchar_t * )buffer.begin();
}

uchar *Memory::read_wstr_as_str( ulong address ) {
    auto name_str = read_wstr( address );
    ulong p = 0;
    wchar_t next;
    do {
        next = name_str[p];
        uchar c = *( uchar * )&next;
        buffer[( uint )p] = c;
        ++p;
    } while ( next != 0 );
    return buffer.begin();
}

uchar *Memory::read_str( ulong address ) {
    uint size = 0;
    uchar c;
    do {
        uc_mem_read( static_cast<uc_engine *>( internal_uc ), address + size, buffer.begin() + size, 1 );
        c = buffer[size];
        ++size;
    } while ( c != 0 && size < BUFFER_SIZE );
    
    if ( size >= BUFFER_SIZE )
        buffer[size - 1] = 0;
    return buffer.begin();
}

void Memory::write_str( ulong address, std::string const &text ) {
    char *buff = ( char * )buffer.begin();
    uint size = ( uint )text.size();
    for ( uint i : urange( size ) )
        buff[i] = text[i];
    buff[size] = 0;
    uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buff, size + 1 );
}

void Memory::write_wstr( ulong address, std::string const &text ) {
    wchar_t *buff = ( wchar_t * )buffer.begin();
    uint size = ( uint )text.size();
    for ( uint i : urange( size ) ) {
        wchar_t t = 0;
        *( char * )&t = text[i];
        buff[i] = t;
    }
    buff[size] = 0;
    uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buff, ( size + 1 ) * sizeof( wchar_t ) );
}

void Memory::write_long_word( ulong address, ulong value ) {
    Utility::write_uint64_t( ( char * )buffer.begin(), value );
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buffer.begin(), 8 ) != UC_ERR_OK )
        Log::err << Log::tag << "Error writing long\n";
}


void Handles::init( Memory &mem ) {
    section = &mem.new_section();
    section->init( MemoryRange( ComputerLayout::HANDLES_ADDRESS, ComputerLayout::HANDLES_RANGE ),
                   "HANDLES", "System",
                   false, false, false
                 );
    section->init_annotations();
    handle_stack.init( section );
}

ulong Handles::get_handle( const char *name ) {
    throw_assert( loaded() && section->has_annotations(), "Handles::get_handle() on uninitialized Handles" );
    auto &note = section->annotations.collection->get_annotation( name, Annotation::HANDLE );
    return 0;
}

ulong Handles::add_handle( const char *name ) {
    throw_assert( loaded() && section->has_annotations(), "Handles::add_handle() on uninitialized Handles" );
    auto handle = handle_stack.get_8byte_slot();
    section->annotations.add_annotation( handle, Annotation( name, Annotation::HANDLE ) );
    return handle;
}


void VirtualHeap::init( Memory &mem, Handles &handles ) {
    heap_size = ComputerLayout::HEAP_SIZE;
    free_map.init( HEAP_BLOCKS );
    size_map.init( HEAP_BLOCKS );
    size_map.set_zero();
    heap_handle = handles.add_handle( "HeapHandle" );
    section = &mem.new_section();
    section->init( MemoryRange( ComputerLayout::HEAP_ADDRESS, ComputerLayout::HEAP_SIZE ),
                   "HEAP", "System",
                   false, true, true );
}

bool VirtualHeap::alloc( ulong size, ulong &address ) {
    throw_assert( loaded(), "VirtualHeap::alloc() on uninitialized VirtualHeap" );
    if ( size == 0 )
        return false;
        
    ulong target_blocks = ( ( size - 1 ) / BLOCK_SIZE ) + 1;
    uint pos = 0;
    ulong count = 0;
    do {
        if ( free_map[( uint )pos] )
            count = 0;
        else
            ++count;
            
        if ( count >= target_blocks ) {
            address = ComputerLayout::HEAP_ADDRESS + ( pos * BLOCK_SIZE );
            size_map[pos] = ( uint )target_blocks;
            for ( auto i : urange( ( uint ) target_blocks ) )
                free_map[pos + i] = true;
            return true;
        }
        ++pos;
    } while ( pos < HEAP_BLOCKS );
    return false;
}

bool VirtualHeap::free( ulong &address ) {
    uint pos = section->address_range.get_local_index( address ) / BLOCK_SIZE;
    for ( auto i : urange( size_map[pos] ) )
        free_map[pos + i] = false;
    size_map[pos] = 0;
    return true;
}

void VirtualStack::init( Memory &mem, Registers &regs ) {
    //Init stack memory
    stack_size = 0x1000 * 0x10;
    
    section = &mem.new_section();
    section->init( MemoryRange( ComputerLayout::STACK_ADDRESS, ( uint )stack_size ),
                   "STACK", "System",
                   false, true, true );
    stack_start = ComputerLayout::STACK_ADDRESS + stack_size - 0x100;
    regs.set_rsp( stack_start );
    this->registers = &regs;
}

ulong VirtualStack::pop_long() {
    throw_assert( loaded(), "VirtualStack::pop_long() on uninitialized VirtualStack." );
    auto rsp = registers->get_rsp(); //Read stack pointer
    uint32_t size = 8;
    auto data = section->read( rsp, size ); //Get memory at stack pointer location
    auto mem_res = Utility::read_uint64_t( ( char * )data ); //Read long word from it
    registers->set_rsp( rsp + 8 ); //Increase stack pointer
    return mem_res;
}

void VirtualStack::push_long( ulong val ) {
    throw_assert( loaded(), "VirtualStack::push_long() on uninitialized VirtualStack." );
    auto rsp = registers->get_rsp() - 8; //Read stack pointer
    uint32_t size = 8;
    auto data = section->mem->buffer.begin();
    Utility::write_uint64_t( ( char * )data, val );
    section->write( rsp, size, data ); //Set memory at stack pointer location
    registers->set_rsp( rsp ); //Decrease stack pointer
}

