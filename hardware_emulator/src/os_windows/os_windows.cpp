#include "os_windows\os_windows.h"
#include "computer/computer_layout.h"
#include <unicorn/unicorn.h>

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


struct DescriptorTable {
    ushort LIMIT_0_15;
    ushort BASE_0_15;
    uchar BASE_16_23;
    uchar TYPE4_S1_DPL2_P1;
    uchar LIMIT16_19_A1_1_DB1_G1;
    uchar BASE_24_31;
};

void OS::Windows::init( Computer &computer ) {
    this->computer = &computer;
    section = &computer.memory.new_section();
    section->init( MemoryRange( ComputerLayout::SYSPAGE_ADDRESS, ComputerLayout::SYSPAGE_RANGE ), "SYSPAGE", "OS",
                   false, true, false );
    section_stack.init( section );
    
    
    //constexpr uint gdt_size = 2;
    //auto gdt_slot = section_stack.get_range( sizeof( DescriptorTable ) * gdt_size );
    //auto tib_slot = section_stack.get_range( sizeof( LOCAL_TIB ) );
    //auto eer_slot = section_stack.get_range( sizeof( LOCAL_EXCEPTION_REGISTRATION_RECORD ) );
    //
    ////Setup TIB
    //LOCAL_TIB thread_info_block;
    //thread_info_block.ExceptionList = ( LOCAL_EXCEPTION_REGISTRATION_RECORD * )eer_slot.start_address;
    //thread_info_block.StackBase = ( void * )( ComputerLayout::STACK_ADDRESS + computer.stack.stack_size );
    //thread_info_block.StackLimit = ( void * )ComputerLayout::STACK_ADDRESS;
    //thread_info_block.SubSystemTib = 0;
    //thread_info_block.ArbitraryUserPointer = 0;
    //thread_info_block.FiberData = 0;
    //thread_info_block.Self = ( LOCAL_TIB * )tib_slot.start_address;
    //LOCAL_EXCEPTION_REGISTRATION_RECORD eer;
    //eer.ExceptionRountineHandler = ( void * )computer.handles.get_handle( "ExceptionRountineHandler" );
    //eer.Next = 0;
    //
    //computer.memory.write_memory( tib_slot.start_address, tib_slot.size, ( uchar * )&thread_info_block );
    //computer.memory.write_memory( eer_slot.start_address, eer_slot.size, ( uchar * )&eer );
    //
    ////Setup Segment Descriptor Table
    //DescriptorTable table[gdt_size];
    //table[0].LIMIT_0_15 = 0xffff;
    //table[0].BASE_0_15 = 0x1000;
    //table[0].BASE_16_23 = 0x0;
    //table[0].TYPE4_S1_DPL2_P1 = 0x9a;
    //table[0].LIMIT16_19_A1_1_DB1_G1 = 0b11001111;
    //table[0].BASE_24_31 = 0x0;
    //
    //table[1].LIMIT_0_15 = 0x0fff;
    //table[1].BASE_0_15 = 0x1000;
    //table[1].BASE_16_23 = 0x0;
    //table[1].TYPE4_S1_DPL2_P1 = 0x9a;
    //table[1].LIMIT16_19_A1_1_DB1_G1 = 0b11000000;
    //table[1].BASE_24_31 = 0x0;
    //
    //computer.memory.write_memory( gdt_slot.start_address, gdt_slot.size, ( uchar * )table );
    //
    ////Set Segment register to link to DescriptorTable
    //uc_x86_mmr v;
    //computer.registers.get_gdtr( &v );
    //std::cout << "GDTR: ";
    //printf( "GDTR: base=%016" PRIx64 " selector=%04" PRIx64 " limit=%08" PRIx64 " flags=%08" PRIx64 "\n",
    //        ( ulong ) v.base, ( ulong )v.selector, ( ulong )v.limit, ( ulong )v.flags );
    //v.base = gdt_slot.start_address;
    //v.limit = gdt_slot.size - 1;
    //computer.registers.set_gdtr( &v );
    //computer.registers.set_gs( 0x1 << 3 );
    //
    //
    ////Set command line
    //std::string cmd_line = "program_name";
    //cmd_line_wstr = section_stack.get_range( ( ( uint )cmd_line.size() + 2 ) * 2 );
    //cmd_line_str = section_stack.get_range( ( uint )cmd_line.size() + 2 );
    //
    //computer.memory.write_wstr( cmd_line_wstr.start_address, cmd_line );
    //computer.memory.write_str( cmd_line_str.start_address, cmd_line );
}

bool OS::Windows::load_dll( const char *file ) {
    if ( !dll.init( file, computer->sys_calls, computer->memory ) )
        return false;
    dll.dll_main( *computer );
    
    return true;
}
