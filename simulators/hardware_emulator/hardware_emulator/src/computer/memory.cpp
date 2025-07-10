/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "computer/memory.h"
#include <unicorn/unicorn.h>
#include "computer/computer_layout.h"
#include "computer/registers.h"
#include <inttypes.h>

#ifdef min
#undef min
#endif
#ifdef max
#undef max
#endif


void AnnotationTable::init() {
    annotations.resize( DEFAULT_ANNOTATION_SIZE );
    annotation_pos = 0;
    new_annotation( 0, Annotation( "NULL-NOTE", Annotation::Type::NONE ) );
}

uint AnnotationTable::new_annotation( ulong base, Annotation const &annotation ) {
    if ( annotation_pos >= annotations.size() )
        annotations.resize( ( annotations.size() ) * 3 / 2 + 1 );
    auto &target = annotations[annotation_pos];
    target = annotation;
    target.base = base;
    return annotation_pos++;
}


void SectionAnnotations::init( AnnotationTable *annotation_table, MemoryRange address_range ) {
    this->annotation_table = annotation_table;
    this->address_range = address_range;
    annotation_id.resize( address_range.size >> 3 );
	std::fill(annotation_id.begin(), annotation_id.end(), 0);
}

void SectionAnnotations::add_annotation( MemoryRange range, Annotation const &annotation ) {
    throw_assert( loaded(), "SectionAnnotations::add_annotation() on uninitialized SectionAnnotations" );
    auto note_id = annotation_table->new_annotation( range.start_address, annotation );
    auto start_index = address_range.get_local_index( range.start_address );
    for ( auto i : urange( start_index >> 3, ( start_index + range.size ) >> 3 ) )
        annotation_id[i] = note_id;
}

void SectionAnnotations::add_annotation( ulong address, Annotation const &annotation ) {
    throw_assert( loaded(), "SectionAnnotations::add_annotation() on uninitialized SectionAnnotations" );
    auto note_id = annotation_table->new_annotation( address, annotation );
    auto index = address_range.get_local_index( address ) >> 3;
    annotation_id[index] = note_id;
}

Annotation *SectionAnnotations::get_annotation( ulong address ) {
    throw_assert( loaded(), "SectionAnnotations::get_annotation() on uninitialized SectionAnnotations" );
    auto note_id = annotation_id[address_range.get_local_index( address ) >> 3];
    if ( note_id != 0 )
        return &( annotation_table->annotations[note_id] );
    return nullptr;
}







bool MemorySection::init( MemoryRange address_range, std::string const &name, std::string const &mod,
                          bool execute, bool read, bool write ) {
    throw_assert( linked(), "MemorySection::init() on unlinked MemorySection." );
    
    this->mapped_range = MemoryRange( 0, 0 );
    this->file_pos = 0;
    this->name = name;
    this->mod = mod;
    this->p_execute = execute;
    this->p_read = read;
    this->p_write = write;
    
    ulong start_address = address_range.start_address;
    ulong end_address = start_address + address_range.size;
    start_address = ( start_address / page_size ) * page_size;
    ulong size = end_address - start_address;
    size = ( ( ( size - 1 ) / ( uint )page_size ) + 1 ) * ( uint )page_size;
    
    this->address_range.start_address = start_address;
    this->address_range.size = ( uint )size;
    
    uint32_t prot = 0;
    if ( execute )
        prot |= UC_PROT_EXEC;
    if ( read )
        prot |= UC_PROT_READ;
    if ( write )
        prot |= UC_PROT_WRITE;
    auto err = uc_mem_map( static_cast<uc_engine *>( internal_uc ), this->address_range.start_address,
                           this->address_range.size, prot );
    if ( err ) {
        
        if (err == UC_ERR_ARG) {
            Log::err.log_tag("Error mapping emulator memory! => Alignment Error");
        }
        else {
            Log::err.log_tag("Error mapping emulator memory!");
        }
        return false;
    }
    return true;
}

bool MemorySection::upload( MemoryRange range, char *data ) {
    throw_assert( loaded(), "MemorySection::upload() on uninitialized MemorySection." );
    throw_assert( linked(), "MemorySection::upload() on unlinked MemorySection." );
    throw_assert( address_range.get_local_index( range.start_address ) + range.size <= address_range.size,
                  "MemorySection::upload() outside section." );
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), range.start_address, data, range.size ) ) {
        Log::err.log_tag("Failed to write emulation code to memory, quit!");
        return false;
    }
    return true;
}


void MemorySection::link( void *internal_uc, ulong page_size, Memory &mem ) {
    this->internal_uc = internal_uc;
    this->page_size = page_size;
    this->mem = &mem;
}

uint64_t MemorySection::address_to_file( uint64_t virtual_address ) {
    throw_assert( loaded(), "MemorySection::address_to_file() on uninitialized memory section." );
    if ( mapped_range.contains( virtual_address ) ) {
        auto local_pos = mapped_range.get_local_index( virtual_address );
        return local_pos + file_pos;
    }
    return virtual_address;
}

void MemorySection::init_annotations() {
    annotations.init( &mem->annotation_table, address_range );
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
MemoryRange SectionStack::get_annotated( uint size, const std::string &name, Annotation::Type type ) {
    auto r = get_range( size );
    if ( mem->has_annotations() )
        mem->annotations.add_annotation( r, Annotation( name, type ) );
    else
        Log::err.log_tag("get_annotated() on section with no annotations");
        
    /*Utility::color_mem_write();
    std::cout << "Added Symbol: " << mod << "!" << name;
    printf( "  %03" PRIx32 "\n", section->address_range.get_local_index( proc_handle.start_address ) );*/
    
    
    //if ( undercorate_function_name( name, name_buffer )
    //        && name.compare( name_buffer.data() ) != 0 ) {
    //    // UnDecorateSymbolName returned success
    //    Utility::color_reg();
    //    printf( "%s\n", name_buffer.data() );
    //}
    return r;
}
uint64_t SectionStack::get_annotated_8byte( const std::string &name, Annotation::Type type, ulong param ) {
    auto r = get_8byte_slot();
    if ( mem->has_annotations() )
        mem->annotations.add_annotation( r, Annotation( name, type, param ) );
    else
        Log::err.log_tag("get_annotated_8byte() on section with no annotations");
    return r;
}
uint64_t SectionStack::get_8byte_slot() {
    throw_assert( loaded(), "SectionStack::get_8byte_slot() on uninitialized SectionStack" );
    auto temp = pos;
    pos += 8;
    return temp + mem->address_range.start_address;
}





void Memory::init( void *uc ) {
    buffer.resize( BUFFER_START_SIZE );
    this->internal_uc = uc;
    section_count = 0;
    uc_query( static_cast<uc_engine *>( internal_uc ), UC_QUERY_PAGE_SIZE, &page_size );
    
    annotation_table.init();
    
    sys_section = &new_section( MemoryRange( ComputerLayout::SYSPAGE_ADDRESS, ComputerLayout::SYSPAGE_RANGE ), "SYSPAGE",
                                "OS", false, true, true );
    sys_section->init_annotations();
    sys_section_stack.init( sys_section );

    exchange_section = &new_section( MemoryRange( ComputerLayout::EXCHANGE_ADDRESS, ComputerLayout::EXCHANGE_RANGE ), "data_exchange",
                                "EMU", false, true, false );
}


void *Memory::read_memory( ulong address, ulong size ) {
    if (size >= buffer.size()) {
        int32_t new_size = ((size / BUFFER_START_SIZE) + 1) * BUFFER_START_SIZE;
        buffer.resize(new_size);
    }
    if ( uc_mem_read( static_cast<uc_engine *>( internal_uc ), address, buffer.data(), size ) ) {
        Log::err.log_tag("Error reading from memory at address %llu with size %llu", address, size);
        buffer[0] = 0;
    }
    return buffer.data();
}

void Memory::write_memory( ulong address, ulong size, void *data ) {
    if (size >= buffer.size()) {
        int32_t new_size = ((size / BUFFER_START_SIZE) + 1) * BUFFER_START_SIZE;
        buffer.resize(new_size);
    }
    uchar *ptr = ( uchar * )data;
    for ( uint i : urange( ( uint )size) )
        buffer[i] = ptr[i];
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buffer.data(), size) )
        Log::err.log_tag("Error writing to memory at address %llu with size %llu", address, size);
}



void Memory::write_memory_buffer(ulong address, ulong size)
{
    if (uc_mem_write(static_cast<uc_engine*>(internal_uc), address, buffer.data(), size))
        Log::err.log_tag("Error writing to memory at address %llu with size %llu", address, size);
}


MemorySection &Memory::new_section( MemoryRange range, const std::string &name, const std::string &module, bool exec,
                                    bool read, bool write ) {
    if ( sections.size() <= section_count )
        sections.resize( sections.size() * 3 / 2 + 2 );
    uint sec_id = section_count++;
    auto sec = new MemorySection();
    sections[sec_id] = std::unique_ptr<MemorySection>( sec );
    sec->link( internal_uc, page_size, *this );
    sec->init( range, name, module, exec, read, write );
    section_lookup[range] = sec_id;
    return *sec;
}

MemorySection *Memory::get_section( ulong virtual_address ) {
    auto res = section_lookup.find( MemoryRange( virtual_address, 0 ) );
    if ( res != section_lookup.end() )
        return sections[res->second].get();
    return nullptr;
}

void MemorySection::print_address_info( ulong virtual_address, char* buffer, int buffer_size) {
    auto file_address = address_to_file( virtual_address );
    int printed = 0;
    if (file_address != virtual_address) {
        printed = snprintf(buffer, buffer_size, "[%s:%s F:%s] ", mod.c_str(), name.c_str(), to_hex(file_address, 16, true).c_str());
    }
    else {
        printed = snprintf(buffer, buffer_size, "[%s:%s] ", mod.c_str(), name.c_str());
    }
    if (printed > 0)
        print_annotation( virtual_address, buffer+printed, buffer_size -printed );
}

void MemorySection::print_annotation( ulong virtual_address, char* buffer, int buffer_size) {
    buffer[0] = '\0';
    if ( has_annotations() ) {
        auto note_ptr = annotations.get_annotation( virtual_address );
        if ( note_ptr ) {
            auto &note = *note_ptr;
            if (note.base != virtual_address) {
                auto sign = note.base < virtual_address ? "" : "-";
                auto val = note.base < virtual_address ? (virtual_address - note.base) : (note.base - virtual_address);
                snprintf(buffer, buffer_size, "(%s[%s%" PRIu64 "])", note.name.c_str(), sign, val);
            }
            else {
                snprintf(buffer, buffer_size, "(%s)", note.name.c_str());
            }
        }
    }
}

void Memory::print_address_info( ulong  virtual_address, char* buffer, int buffer_size ) {
    auto sec_ptr = get_section( virtual_address );
    if ( sec_ptr )
        sec_ptr->print_address_info( virtual_address, buffer, buffer_size );
    else
        sprintf(buffer, "[NON-ALLOCATED] ");
}

void Memory::print_annotation( ulong virtual_address, char* buffer, int buffer_size) {
    auto sec_ptr = get_section( virtual_address );
    if ( sec_ptr )
        sec_ptr->print_annotation( virtual_address, buffer, buffer_size);
}



wchar_t *Memory::read_wstr( ulong address ) {
    auto buff_size = buffer.size();
    uint size = 0;
    wchar_t* target = (wchar_t* )buffer.data();
    do {
        if ( uc_mem_read( static_cast<uc_engine *>( internal_uc ), address + size, target, 2 ) ) {
            Log::err.log_tag("Error reading wstr at address %llu", address);
            *target = 0;
            break;
        }
        ++target;
        size += 2;
        if (size >= buff_size) {
            buff_size += BUFFER_START_SIZE;
            buffer.resize(buff_size);
        }
    } while ( *target != 0 && size < MAX_BUFFER_SIZE);
    *target = 0;
    return ( wchar_t * )buffer.data();
}

char *Memory::read_wstr_as_str( ulong address ) {
    throw_assert(false, "TODO correct");
    auto name_str = read_wstr( address );
    ulong p = 0;
    wchar_t next;
    do {
        next = name_str[p];
        uchar c = *( uchar * )&next;
        buffer[( uint )p] = c;
        ++p;
    } while ( next != 0 );
    return (char*)buffer.data();
}

char *Memory::read_str( ulong address ) {
    uint size = 0;
    auto buff_size = buffer.size();
    do {
        if (size >= buff_size){
            buff_size += BUFFER_START_SIZE;
            buffer.resize(buff_size);
        }
        if ( uc_mem_read( static_cast<uc_engine *>( internal_uc ), address + size, buffer.data() + size, 1 ) ) {
            Log::err.log_tag("Error reading str at address %llu", address);
            buffer[size] = 0;
            break;
        }
    } while ( buffer[size++] != 0 && size < MAX_BUFFER_SIZE );
    
    if ( size >= MAX_BUFFER_SIZE )
        buffer[size - 1] = 0;
    return (char*)buffer.data();
}


void Memory::write_str( ulong address, std::string const &text ) {
    char *buff = ( char * )buffer.data();
    uint size = std::min(( uint )text.size(), (uint)buffer.size());
    for ( uint i : urange( size ) )
        buff[i] = text[i];
    buff[size] = 0;
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buff, size + 1LL ) )
        Log::err.log_tag("Error writing str at address %llu", address);
}

void Memory::write_wstr( ulong address, std::string const &text ) {
    wchar_t *buff = ( wchar_t * )buffer.data();
    uint size = ( uint )text.size();
    for ( uint i : urange( size ) ) {
        wchar_t t = 0;
        *( char * )&t = text[i];
        buff[i] = t;
    }
    buff[size] = 0;
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buff, ( size + 1LL ) * sizeof( wchar_t ) ) )
        Log::err.log_tag("Error writing wstr at address %llu", address);
}

void Memory::write_long_word( ulong address, ulong value ) {
    Utility::write_uint64_t( ( char * )buffer.data(), value );
    if ( uc_mem_write( static_cast<uc_engine *>( internal_uc ), address, buffer.data(), 8 ) != UC_ERR_OK )
        Log::err.log_tag("Error writing long");
}



MemAccess Memory::get_mem_access( uint type ) {
    switch ( type ) {
        case UC_MEM_WRITE: case UC_MEM_WRITE_UNMAPPED: case UC_MEM_WRITE_PROT:
            return MemAccess::WRITE;
        case UC_MEM_READ: case UC_MEM_READ_UNMAPPED: case UC_MEM_READ_PROT:
            return MemAccess::READ;
        case UC_MEM_FETCH: case UC_MEM_FETCH_UNMAPPED: case UC_MEM_FETCH_PROT:
            return MemAccess::FETCH;
        default:
            return MemAccess::NONE;
    }
}

MemAccessError Memory::get_mem_err( uint type ) {
    switch ( type ) {
        case UC_MEM_WRITE: case UC_MEM_READ: case UC_MEM_FETCH:
            return MemAccessError::NONE;
        case UC_MEM_WRITE_UNMAPPED: case UC_MEM_READ_UNMAPPED: case UC_MEM_FETCH_UNMAPPED:
            return MemAccessError::MAPPED;
        case UC_MEM_WRITE_PROT: case UC_MEM_READ_PROT: case UC_MEM_FETCH_PROT:
            return MemAccessError::PROT;
        default:
            return MemAccessError::NONE;
    }
}


void Handles::init( Memory &mem ) {
    section = &mem.new_section( MemoryRange( ComputerLayout::HANDLES_ADDRESS, ComputerLayout::HANDLES_RANGE ),
                                "HANDLES", "System",
                                false, false, false
                              );
    section->init_annotations();
    handle_stack.init( section );
}

//ulong Handles::get_handle( const char *name ) {
//    throw_assert( loaded() && section->has_annotations(), "Handles::get_handle() on uninitialized Handles" );
//    auto &note = section->annotations.collection->get_annotation( name, Annotation::HANDLE );
//    return 0;
//}

ulong Handles::add_handle( const char *name ) {
    throw_assert( loaded() && section->has_annotations(), "Handles::add_handle() on uninitialized Handles" );
    auto handle = handle_stack.get_annotated_8byte( name, Annotation::Type::HANDLE );
    return handle;
}


void VirtualHeap::init( Memory &mem, Handles &handles ) {
    heap_size = ComputerLayout::HEAP_SIZE;
    free_map.resize( HEAP_BLOCKS );
    size_map.resize( HEAP_BLOCKS );
	std::fill(size_map.begin(), size_map.end(), 0);
    heap_handle = handles.add_handle( "HeapHandle" );
    section = &mem.new_section(
                  MemoryRange( ComputerLayout::HEAP_ADDRESS, ComputerLayout::HEAP_SIZE ),
                  "HEAP", "System",
                  false, true, true
              );
}

bool VirtualHeap::alloc( ulong size, ulong &address ) {
    throw_assert( loaded(), "VirtualHeap::alloc() on uninitialized VirtualHeap" );
    if ( size == 0 )
        return false;
        
    ulong target_blocks = ( ( size - 1 ) / BLOCK_SIZE ) + 1;
    uint pos = 0;
    ulong count = 0;
    do {
        // search free pos
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
    
    section = &mem.new_section(
                  MemoryRange( ComputerLayout::STACK_ADDRESS + ComputerLayout::STACK_MAX_SIZE - stack_size,
                               ( uint )stack_size ),
                  "STACK", "System",
                  false, true, true
              );
    stack_start = ComputerLayout::STACK_ADDRESS + ComputerLayout::STACK_MAX_SIZE - 0x100;
    regs.set_rsp( stack_start );
    this->registers = &regs;
    regs.set_rsp( stack_start );
}

ulong VirtualStack::pop_long() {
    throw_assert( loaded(), "VirtualStack::pop_long() on uninitialized VirtualStack." );
    uint32_t size = 8;
    auto rsp = registers->get_rsp(); //Read stack pointer
    auto data = section->mem->read_memory( MemoryRange( rsp, size ) ); //Get memory at stack pointer location
    auto mem_res = Utility::read_uint64_t( ( char * )data ); //Read long word from it
    registers->set_rsp( rsp + size ); //Increase stack pointer
    return mem_res;
}

void VirtualStack::push_long( ulong val ) {
    throw_assert( loaded(), "VirtualStack::push_long() on uninitialized VirtualStack." );
    uint32_t size = 8;
    auto rsp = registers->get_rsp() - size; //Read stack pointer
    auto data = section->mem->buffer.data();
    Utility::write_uint64_t( ( char * )data, val );
    section->mem->write_memory( MemoryRange( rsp, size ), data ); //Set memory at stack pointer location
    registers->set_rsp( rsp ); //Decrease stack pointer
}

uint VirtualStack::pop_int() {
    throw_assert( loaded(), "VirtualStack::pop_int() on uninitialized VirtualStack." );
    uint32_t size = 4;
    auto rsp = registers->get_rsp(); //Read stack pointer
    auto data = section->mem->read_memory( MemoryRange( rsp, size ) ); //Get memory at stack pointer location
    auto mem_res = Utility::read_uint64_t( ( char * )data ); //Read long word from it
    registers->set_rsp( rsp + size ); //Increase stack pointer
    return ( uint ) mem_res;
}

void VirtualStack::push_int( uint val ) {
    throw_assert( loaded(), "VirtualStack::push_int() on uninitialized VirtualStack." );
    uint32_t size = 4;
    auto rsp = registers->get_rsp() - size; //Read stack pointer
    auto data = section->mem->buffer.data();
    Utility::write_uint32_t( ( char * )data, val );
    section->mem->write_memory( MemoryRange( rsp, size ), data ); //Set memory at stack pointer location
    registers->set_rsp( rsp ); //Decrease stack pointer
}

