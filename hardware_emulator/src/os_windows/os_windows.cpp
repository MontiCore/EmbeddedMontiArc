#include "os_windows/os_windows.h"
#include "computer/computer_layout.h"
#include <unicorn/unicorn.h>

MemoryRange OS::io_slot;

struct LOCAL_EXCEPTION_REGISTRATION_RECORD {
    LOCAL_EXCEPTION_REGISTRATION_RECORD *Next;
    void *ExceptionRountineHandler;
};

struct LOCAL_TIB {
    LOCAL_EXCEPTION_REGISTRATION_RECORD *ExceptionList;
    void *StackBase;
    void *StackLimit;
    void *SubSystemTib;
    void *FiberData;
    void *ArbitraryUserPointer;
    LOCAL_TIB *Self;
};


struct SegmentDescriptor {
    /*
        See https://wiki.osdev.org/Global_Descriptor_Table
    */
    enum BIT_POS {
        Pr = 7,
        Privl = 5,
        S = 4,
        Ex = 3,
        DC = 2,
        RW = 1,
        Ac = 0,
        Gr = 7,
        Sz = 6,
    };
    union {
        struct {
            ushort LIMIT_0_15;
            ushort BASE_0_15;
            uchar BASE_16_23;
            uchar ACCESS_BYTE;
            uchar LIMIT16_19_FLAGS;
            uchar BASE_24_31;
        };
        uchar data[8];
    };
    
    
    void set_standard_segment( ulong addr, ulong limit, bool granularity ) {
        set_zero();
        set_limit( 0xFFFFF );
        set_base( addr );
        set_present();
        set_descriptor();
        set_executable( false, true );
        if ( granularity )
            set_granularity_4kb();
        set_size_32();
    }
    
    
    
    /*
        Set zero before setting up value
    */
    void set_zero() {
        for ( auto i : urange( 8 ) )
            data[i] = 0;
    }
    
    void set_base( ulong base ) {
        BASE_0_15 = ( ushort )( base & BIT_MASKS[16] );
        BASE_16_23 = ( uchar )( ( base >> 16 ) & BIT_MASKS[8] );
        BASE_24_31 = ( uchar )( ( base >> 24 ) & BIT_MASKS[8] );
    }
    void set_limit( ulong limit ) {
        LIMIT_0_15 = ( ushort )( limit & BIT_MASKS[16] );
        LIMIT16_19_FLAGS |= ( ushort )( ( limit >> 16 ) & BIT_MASKS[4] );
    }
    void set_present() {
        setBitHigh( ACCESS_BYTE, Pr );
    }
    void set_privilege( uchar priv ) {
        ACCESS_BYTE |= ( priv & BIT_MASKS[2] ) << Privl;
    }
    //Set false for system segments
    void set_descriptor() {
        setBitHigh( ACCESS_BYTE, S );
    }
    void set_executable( bool confirming, bool readable ) {
        setBitHigh( ACCESS_BYTE, Ex );
        if ( confirming )
            setBitHigh( ACCESS_BYTE, DC );
        if ( readable )
            setBitHigh( ACCESS_BYTE, RW );
    }
    void set_data( bool grows_down, bool writable ) {
        if ( grows_down )
            setBitHigh( ACCESS_BYTE, DC );
        if ( writable )
            setBitHigh( ACCESS_BYTE, RW );
    }
    void set_granularity_4kb() {
        setBitHigh( LIMIT16_19_FLAGS, Gr );
    }
    void set_size_32() {
        setBitHigh( LIMIT16_19_FLAGS, Sz );
    }
};

void OS::Windows::init( Computer &computer ) {
    this->computer = &computer;
    section = computer.memory.sys_section;
    section_stack = &computer.memory.sys_section_stack;
    
    
    constexpr uint gdt_size = 2;
    auto gdt_slot = section_stack->get_range( sizeof( SegmentDescriptor ) * gdt_size );
    auto tib_slot = section_stack->get_range( sizeof( LOCAL_TIB ) );
    auto eer_slot = section_stack->get_range( sizeof( LOCAL_EXCEPTION_REGISTRATION_RECORD ) );
    
    //Setup TIB
    LOCAL_TIB thread_info_block;
    thread_info_block.ExceptionList = ( LOCAL_EXCEPTION_REGISTRATION_RECORD * )eer_slot.start_address;
    thread_info_block.StackBase = ( void * )( ComputerLayout::STACK_ADDRESS + computer.stack.stack_size );
    thread_info_block.StackLimit = ( void * )ComputerLayout::STACK_ADDRESS;
    thread_info_block.SubSystemTib = 0;
    thread_info_block.ArbitraryUserPointer = 0;
    thread_info_block.FiberData = 0;
    thread_info_block.Self = ( LOCAL_TIB * )tib_slot.start_address;
    LOCAL_EXCEPTION_REGISTRATION_RECORD eer;
    eer.ExceptionRountineHandler = ( void * )computer.handles.get_handle( "ExceptionRountineHandler" );
    eer.Next = 0;
    
    computer.memory.write_memory( tib_slot, ( uchar * )&thread_info_block );
    section->annotations.add_annotation( tib_slot, Annotation( "Thread Information Block", Annotation::SYMBOL ) );
    computer.memory.write_memory( eer_slot, ( uchar * )&eer );
    section->annotations.add_annotation( eer_slot, Annotation( "Exception Registration Record", Annotation::SYMBOL ) );
    
    //Setup Segment Descriptor Table
    SegmentDescriptor table[gdt_size];
    table[0].set_standard_segment( 0, 0xFFFFF, true );
    table[1].set_standard_segment( tib_slot.start_address, 0xFFF, false );
    
    computer.memory.write_memory( gdt_slot, ( uchar * )table );
    section->annotations.add_annotation( gdt_slot, Annotation( "Global Descriptor Table", Annotation::SYMBOL ) );
    
    //Set Segment register to link to DescriptorTable
    uc_x86_mmr v;
    computer.registers.get_gdtr( &v );
    /*std::cout << "GDTR: ";
    printf( "GDTR: base=%016" PRIx64 " selector=%04" PRIx64 " limit=%08" PRIx64 " flags=%08" PRIx64 "\n",
            ( ulong ) v.base, ( ulong )v.selector, ( ulong )v.limit, ( ulong )v.flags );*/
    v.base = gdt_slot.start_address;
    v.limit = gdt_slot.size - 1;
    computer.registers.set_gdtr( &v );
    computer.registers.set_gs( 0x1 << 3 );
    
    
    //Set command line
    std::string cmd_line = "program_name";
    cmd_line_wstr = section_stack->get_range( ( ( uint )cmd_line.size() + 2 ) * 2 );
    cmd_line_str = section_stack->get_range( ( uint )cmd_line.size() + 2 );
    
    computer.memory.write_wstr( cmd_line_wstr.start_address, cmd_line );
    section->annotations.add_annotation( cmd_line_wstr, Annotation( "Command Line WSTR", Annotation::SYMBOL ) );
    computer.memory.write_str( cmd_line_str.start_address, cmd_line );
    section->annotations.add_annotation( cmd_line_str, Annotation( "Command Line STR", Annotation::SYMBOL ) );
    
    auto cout_ptr_addr = add_symbol( "MSVCP140.DLL", "?cout@std@@3V?$basic_ostream@DU?$char_traits@D@std@@@1@A",
                                     8, Annotation::PROC );
    auto cout_slot = section_stack->get_range( sizeof( std::cout ) );
    computer.memory.write_memory( cout_ptr_addr, 8, ( uchar * )&cout_slot.start_address );
    section->annotations.add_annotation( cout_ptr_addr, Annotation( "Cout pointer", Annotation::SYMBOL ) );
    section->annotations.add_annotation( cout_slot, Annotation( "STD::COUT", Annotation::SYMBOL ) );
    
    typedef struct {
        short level;
        short token;
        short bsize;
        char fd;
        unsigned flags;
        unsigned char hold;
        unsigned char *buffer;
        unsigned char *curp;
        unsigned istemp;
    } FILE;
    FILE io_files[3] = {};
    io_slot = section_stack->get_range( sizeof( io_files ) );
    io_stdin = MemoryRange( io_slot.start_address, sizeof( FILE ) );
    io_stdout = MemoryRange( io_slot.start_address + sizeof( FILE ), sizeof( FILE ) );
    io_stderr = MemoryRange( io_slot.start_address + sizeof( FILE ) * 2, sizeof( FILE ) );
    section->annotations.add_annotation( io_stdin, Annotation( "io_stdin", Annotation::SYMBOL ) );
    section->annotations.add_annotation( io_stdout, Annotation( "io_stdout", Annotation::SYMBOL ) );
    section->annotations.add_annotation( io_stderr, Annotation( "io_stderr", Annotation::SYMBOL ) );
    computer.memory.write_memory( io_slot, ( uchar * )io_files );
}

bool OS::Windows::load_file( const char *file ) {
    if ( !dll.init( file, computer->sys_calls, computer->memory ) )
        return false;
    dll.dll_main( *computer );
    
    return true;
}

ulong OS::Windows::add_symbol( const std::string &mod, const std::string &name, uint size, Annotation::Type type ) {
    std::string res_name = mod + "!" + name;
    auto proc_handle = section_stack->get_range( size );
    section->annotations.add_annotation( proc_handle, Annotation( res_name, type ) );
    
    /*Utility::color_mem_write();
    std::cout << "Added Symbol: " << mod << "!" << name;
    printf( "  %03" PRIx32 "\n", section->address_range.get_local_index( proc_handle.start_address ) );*/
    
    
    if ( undercorate_function_name( name, name_buffer )
            && name.compare( name_buffer.begin() ) != 0 ) {
        // UnDecorateSymbolName returned success
        /*Utility::color_reg();
        printf( "%s\n", name_buffer.begin() );*/
    }
    return proc_handle.start_address;
}
