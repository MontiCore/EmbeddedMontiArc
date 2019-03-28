#include "os_windows/os_windows.h"
//#include "computer/computer_layout.h"
#include <unicorn/unicorn.h>
#include "windows_calls.h"

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
    auto gdt_slot = section_stack->get_annotated( sizeof( SegmentDescriptor ) * gdt_size, "Global Descriptor Table",
                    Annotation::OBJECT );
    auto tib_slot = section_stack->get_annotated( sizeof( LOCAL_TIB ), "Thread Information Block", Annotation::OBJECT );
    auto eer_slot = section_stack->get_annotated( sizeof( LOCAL_EXCEPTION_REGISTRATION_RECORD ),
                    "Exception Registration Record", Annotation::OBJECT );
                    
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
    eer.ExceptionRountineHandler = ( void * )computer.handles.add_handle( "ExceptionRountineHandler" );
    eer.Next = 0;
    
    computer.memory.write_memory( tib_slot, ( uchar * )&thread_info_block );
    computer.memory.write_memory( eer_slot, ( uchar * )&eer );
    
    //Setup Segment Descriptor Table
    SegmentDescriptor table[gdt_size];
    table[0].set_standard_segment( 0, 0xFFFFF, true );
    table[1].set_standard_segment( tib_slot.start_address, 0xFFF, false );
    
    computer.memory.write_memory( gdt_slot, ( uchar * )table );
    
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
    cmd_line_wstr = section_stack->get_annotated( ( ( uint )cmd_line.size() + 2 ) * 2, "Command Line WSTR",
                    Annotation::OBJECT );
    cmd_line_str = section_stack->get_annotated( ( uint )cmd_line.size() + 2, "Command Line STR", Annotation::OBJECT );
    
    computer.memory.write_wstr( cmd_line_wstr.start_address, cmd_line );
    computer.memory.write_str( cmd_line_str.start_address, cmd_line );
    
    auto cout_ptr_slot = section_stack->get_annotated( 8, "std::cout ptr", Annotation::OBJECT );
    computer.symbols.add_symbol( "?cout@std@@3V?$basic_ostream@DU?$char_traits@D@std@@@1@A", Symbols::Symbol::OBJECT,
                                 cout_ptr_slot.start_address );
                                 
    auto cout_slot = section_stack->get_annotated( sizeof( std::cout ), "STD::COUT", Annotation::OBJECT );
    computer.memory.write_memory( cout_ptr_slot, ( uchar * )&cout_slot.start_address );
    
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
    io_stdin = section_stack->get_annotated( sizeof( FILE ), "io_stdin", Annotation::OBJECT );
    io_stdout = section_stack->get_annotated( sizeof( FILE ), "io_stdout", Annotation::OBJECT );
    io_stderr = section_stack->get_annotated( sizeof( FILE ), "io_stderr", Annotation::OBJECT );
    computer.memory.write_memory( io_stdin, &io_files[0] );
    computer.memory.write_memory( io_stdout, &io_files[1] );
    computer.memory.write_memory( io_stderr, &io_files[2] );
    computer.io_slot.start_address = io_stdin.start_address;
    
    WindowsCalls::add_windows_calls( computer.sys_calls, *this );
    computer.func_call = std::unique_ptr<FunctionCalling>( new WindowsFastCall( computer.registers ) );
}

bool OS::Windows::load_file( const char *file ) {
    if ( !dll.init( std::string( file ) + ".dll", computer->sys_calls, computer->memory, computer->symbols ) )
        return false;
    dll.dll_main( *computer );
    
    return true;
}


namespace OS {

    //Caller
    void WindowsFastCall::set_params_64( ulong p1 ) {
        set_param1_64( p1 );
    }
    void WindowsFastCall::set_params_64( ulong p1, ulong p2 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
    }
    void WindowsFastCall::set_params_64( ulong p1, ulong p2, ulong p3 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
        set_param3_64( p3 );
    }
    void WindowsFastCall::set_params_64( ulong p1, ulong p2, ulong p3, ulong p4 ) {
        set_param1_64( p1 );
        set_param2_64( p2 );
        set_param3_64( p3 );
        set_param4_64( p4 );
    }
    void WindowsFastCall::set_params_32( uint p1 ) {
        set_param1_32( p1 );
    }
    void WindowsFastCall::set_params_32( uint p1, uint p2 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
    }
    void WindowsFastCall::set_params_32( uint p1, uint p2, uint p3 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
        set_param3_32( p3 );
    }
    void WindowsFastCall::set_params_32( uint p1, uint p2, uint p3, uint p4 ) {
        set_param1_32( p1 );
        set_param2_32( p2 );
        set_param3_32( p3 );
        set_param4_32( p4 );
    }
    void WindowsFastCall::set_params_double( double p1 ) {
        set_param1_double( p1 );
    }
    void WindowsFastCall::set_params_double( double p1, double p2 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
    }
    void WindowsFastCall::set_params_double( double p1, double p2, double p3 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
        set_param3_double( p3 );
    }
    void WindowsFastCall::set_params_double( double p1, double p2, double p3, double p4 ) {
        set_param1_double( p1 );
        set_param2_double( p2 );
        set_param3_double( p3 );
        set_param4_double( p4 );
    }
    void WindowsFastCall::set_params_float( float p1 ) {
        set_param1_float( p1 );
    }
    void WindowsFastCall::set_params_float( float p1, float p2 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
    }
    void WindowsFastCall::set_params_float( float p1, float p2, float p3 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
        set_param3_float( p3 );
    }
    void WindowsFastCall::set_params_float( float p1, float p2, float p3, float p4 ) {
        set_param1_float( p1 );
        set_param2_float( p2 );
        set_param3_float( p3 );
        set_param4_float( p4 );
    }
    void WindowsFastCall::set_param1_64( ulong p ) {
        registers.set_rcx( p );
    }
    void WindowsFastCall::set_param2_64( ulong p ) {
        registers.set_rdx( p );
    }
    void WindowsFastCall::set_param3_64( ulong p ) {
        registers.set_r8( p );
    }
    void WindowsFastCall::set_param4_64( ulong p ) {
        registers.set_r9( p );
    }
    void WindowsFastCall::set_param1_32( uint p ) {
        registers.set_rcx( p );
    }
    void WindowsFastCall::set_param2_32( uint p ) {
        registers.set_rdx( p );
    }
    void WindowsFastCall::set_param3_32( uint p ) {
        registers.set_r8( p );
    }
    void WindowsFastCall::set_param4_32( uint p ) {
        registers.set_r9( p );
    }
    void WindowsFastCall::set_param1_double( double p ) {
        registers.set_xmm0( p );
    }
    void WindowsFastCall::set_param2_double( double p ) {
        registers.set_xmm1( p );
    }
    void WindowsFastCall::set_param3_double( double p ) {
        registers.set_xmm2( p );
    }
    void WindowsFastCall::set_param4_double( double p ) {
        registers.set_xmm3( p );
    }
    void WindowsFastCall::set_param1_float( float p ) {
        registers.set_xmm0_f( p );
    }
    void WindowsFastCall::set_param2_float( float p ) {
        registers.set_xmm1_f( p );
    }
    void WindowsFastCall::set_param3_float( float p ) {
        registers.set_xmm2_f( p );
    }
    void WindowsFastCall::set_param4_float( float p ) {
        registers.set_xmm3_f( p );
    }
    ulong WindowsFastCall::get_return_64() {
        return registers.get_rax();
    }
    uint WindowsFastCall::get_return_32() {
        return ( uint )registers.get_rax();
    }
    double WindowsFastCall::get_return_double() {
        return registers.get_xmm0();
    }
    
    float WindowsFastCall::get_return_float() {
        return registers.get_xmm0_f();
    }
    
    char WindowsFastCall::get_return_char() {
        return ( char )registers.get_rax();
    }
    
    
    //Callee
    ulong WindowsFastCall::get_param1_64() {
        return registers.get_rcx();
    }
    ulong WindowsFastCall::get_param2_64() {
        return registers.get_rdx();
    }
    ulong WindowsFastCall::get_param3_64() {
        return registers.get_r8();
    }
    ulong WindowsFastCall::get_param4_64() {
        return registers.get_r9();
    }
    uint WindowsFastCall::get_param1_32() {
        return ( uint )registers.get_rcx();
    }
    uint WindowsFastCall::get_param2_32() {
        return ( uint )registers.get_rdx();
    }
    uint WindowsFastCall::get_param3_32() {
        return ( uint )registers.get_r8();
    }
    uint WindowsFastCall::get_param4_32() {
        return ( uint )registers.get_r9();
    }
    double WindowsFastCall::get_param1_double() {
        return registers.get_xmm0();
    }
    double WindowsFastCall::get_param2_double() {
        return registers.get_xmm1();
    }
    double WindowsFastCall::get_param3_double() {
        return registers.get_xmm2();
    }
    double WindowsFastCall::get_param4_double() {
        return registers.get_xmm3();
    }
    float WindowsFastCall::get_param1_float() {
        return registers.get_xmm0_f();
    }
    float WindowsFastCall::get_param2_float() {
        return registers.get_xmm1_f();
    }
    float WindowsFastCall::get_param3_float() {
        return registers.get_xmm2_f();
    }
    float WindowsFastCall::get_param4_float() {
        return registers.get_xmm3_f();
    }
    void WindowsFastCall::set_return_64( ulong r ) {
        registers.set_rax( r );
    }
    void WindowsFastCall::set_return_32( uint r ) {
        registers.set_rax( r );
    }
    void WindowsFastCall::set_return_double( double r ) {
        registers.set_xmm0( r );
    }
}