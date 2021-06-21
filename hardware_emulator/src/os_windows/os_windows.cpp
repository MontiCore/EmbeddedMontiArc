/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "os_windows/os_windows.h"
//#include "computer/computer_layout.h"
#include <unicorn/unicorn.h>
#include "windows_system_calls.h"
#include <iostream>


void OS::Windows::load_file(const fs::path& file) {
    dll.init(file.parent_path() / (file.stem().string() + ".dll"), computer->sys_calls, computer->memory, computer->symbols);
    dll.dll_main(*computer);
}




void OS::Windows::init( Computer &computer ) {
    this->computer = &computer;
    section = computer.memory.sys_section;
    section_stack = &computer.memory.sys_section_stack;
    
    setup_tib_gdt();
    setup_command_line_args();
    setup_cout();
    setup_io();
    setup_locale();
    computer.add_symbol_handle("msvcrt.dll");
    
    WindowsSystemCalls::add_windows_calls( computer.sys_calls, *this );
}


struct LOCAL_EXCEPTION_REGISTRATION_RECORD {
    LOCAL_EXCEPTION_REGISTRATION_RECORD* Next;
    void* ExceptionRountineHandler;
};

struct LOCAL_TIB {
    LOCAL_EXCEPTION_REGISTRATION_RECORD* ExceptionList;
    void* StackBase;
    void* StackLimit;
    void* SubSystemTib;
    void* FiberData;
    void* ArbitraryUserPointer;
    LOCAL_TIB* Self;
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


    void set_standard_segment(ulong addr, ulong limit, bool granularity) {
        set_zero();
        set_limit(0xFFFFF);
        set_base(addr);
        set_present();
        set_descriptor();
        set_executable(false, true);
        if (granularity)
            set_granularity_4kb();
        set_size_32();
    }



    /*
        Set zero before setting up value
    */
    void set_zero() {
        for (auto i : urange(8))
            data[i] = 0;
    }

    void set_base(ulong base) {
        BASE_0_15 = (ushort)(base & BIT_MASKS[16]);
        BASE_16_23 = (uchar)((base >> 16)& BIT_MASKS[8]);
        BASE_24_31 = (uchar)((base >> 24)& BIT_MASKS[8]);
    }
    void set_limit(ulong limit) {
        LIMIT_0_15 = (ushort)(limit & BIT_MASKS[16]);
        LIMIT16_19_FLAGS |= (ushort)((limit >> 16)& BIT_MASKS[4]);
    }
    void set_present() {
        setBitHigh(ACCESS_BYTE, Pr);
    }
    void set_privilege(uchar priv) {
        ACCESS_BYTE |= (priv & BIT_MASKS[2]) << Privl;
    }
    //Set false for system segments
    void set_descriptor() {
        setBitHigh(ACCESS_BYTE, S);
    }
    void set_executable(bool confirming, bool readable) {
        setBitHigh(ACCESS_BYTE, Ex);
        if (confirming)
            setBitHigh(ACCESS_BYTE, DC);
        if (readable)
            setBitHigh(ACCESS_BYTE, RW);
    }
    void set_data(bool grows_down, bool writable) {
        if (grows_down)
            setBitHigh(ACCESS_BYTE, DC);
        if (writable)
            setBitHigh(ACCESS_BYTE, RW);
    }
    void set_granularity_4kb() {
        setBitHigh(LIMIT16_19_FLAGS, Gr);
    }
    void set_size_32() {
        setBitHigh(LIMIT16_19_FLAGS, Sz);
    }
};

void OS::Windows::setup_tib_gdt()
{


    constexpr uint gdt_size = 2;
    auto gdt_slot = section_stack->get_annotated(sizeof(SegmentDescriptor) * gdt_size, "Global Descriptor Table",
        Annotation::Type::OBJECT);
    auto tib_slot = section_stack->get_annotated(sizeof(LOCAL_TIB), "Thread Information Block", Annotation::Type::OBJECT);
    auto eer_slot = section_stack->get_annotated(sizeof(LOCAL_EXCEPTION_REGISTRATION_RECORD),
        "Exception Registration Record", Annotation::Type::OBJECT);

    //Setup TIB
    LOCAL_TIB thread_info_block;
    thread_info_block.ExceptionList = (LOCAL_EXCEPTION_REGISTRATION_RECORD*)eer_slot.start_address;
    thread_info_block.StackBase = (void*)(ComputerLayout::STACK_ADDRESS + computer->stack.stack_size);
    thread_info_block.StackLimit = (void*)ComputerLayout::STACK_ADDRESS;
    thread_info_block.SubSystemTib = 0;
    thread_info_block.ArbitraryUserPointer = 0;
    thread_info_block.FiberData = 0;
    thread_info_block.Self = (LOCAL_TIB*)tib_slot.start_address;
    LOCAL_EXCEPTION_REGISTRATION_RECORD eer;
    eer.ExceptionRountineHandler = (void*)computer->handles.add_handle("ExceptionRountineHandler");
    eer.Next = 0;

    computer->memory.write_memory(tib_slot, (uchar*)&thread_info_block);
    computer->memory.write_memory(eer_slot, (uchar*)&eer);

    //Setup Segment Descriptor Table
    SegmentDescriptor table[gdt_size] = { 0 };
    table[0].set_standard_segment(0, 0xFFFFF, true);
    table[1].set_standard_segment(tib_slot.start_address, 0xFFF, false);

    computer->memory.write_memory(gdt_slot, (uchar*)table);

    //Set Segment register to link to DescriptorTable
    uc_x86_mmr v;
    computer->registers.get_gdtr(&v);
    /*std::cout << "GDTR: ";
    printf( "GDTR: base=%016" PRIx64 " selector=%04" PRIx64 " limit=%08" PRIx64 " flags=%08" PRIx64 "\n",
            ( ulong ) v.base, ( ulong )v.selector, ( ulong )v.limit, ( ulong )v.flags );*/
    v.base = gdt_slot.start_address;
    v.limit = gdt_slot.size - 1;
    computer->registers.set_gdtr(&v);
    computer->registers.set_gs(0x1 << 3);
}

void OS::Windows::setup_command_line_args()
{

    //Set command line    
    std::string cmd_line = "program_name";
    cmd_line_wstr = section_stack->get_annotated(((uint)cmd_line.size() + 2) * 2, "Command Line WSTR",
        Annotation::Type::OBJECT);
    cmd_line_str = section_stack->get_annotated((uint)cmd_line.size() + 2, "Command Line STR", Annotation::Type::OBJECT);

    computer->memory.write_wstr(cmd_line_wstr.start_address, cmd_line);
    computer->memory.write_str(cmd_line_str.start_address, cmd_line);

}

void OS::Windows::setup_cout()
{
    // Set dummy cout handle
    auto cout_ptr_slot = section_stack->get_annotated(16, "std::cout ptr", Annotation::Type::OBJECT);
    computer->symbols.add_symbol("?cout@std@@3V?$basic_ostream@DU?$char_traits@D@std@@@1@A", Symbols::Symbol::Type::OBJECT,
        cout_ptr_slot.start_address);
    computer->symbols.add_symbol("__fu0__ZSt4cout", Symbols::Symbol::Type::OBJECT,
        cout_ptr_slot.start_address + 8);


    auto cout_slot = section_stack->get_annotated(sizeof(std::cout), "STD::COUT", Annotation::Type::OBJECT);
    computer->memory.write_memory(cout_ptr_slot, (uchar*)&cout_slot.start_address);
}

void OS::Windows::setup_io()
{
    // Set IO structs
    typedef struct {
        short level;
        short token;
        short bsize;
        char fd;
        unsigned flags;
        unsigned char hold;
        unsigned char* buffer;
        unsigned char* curp;
        unsigned istemp;
    } FILE;
    FILE io_files[3] = {};
    io_stdin = section_stack->get_annotated(sizeof(FILE), "io_stdin", Annotation::Type::OBJECT);
    io_stdout = section_stack->get_annotated(sizeof(FILE), "io_stdout", Annotation::Type::OBJECT);
    io_stderr = section_stack->get_annotated(sizeof(FILE), "io_stderr", Annotation::Type::OBJECT);
    computer->memory.write_memory(io_stdin, &io_files[0]);
    computer->memory.write_memory(io_stdout, &io_files[1]);
    computer->memory.write_memory(io_stderr, &io_files[2]);
    computer->io_slot.start_address = io_stdin.start_address;
}

void OS::Windows::setup_locale()
{
    // Set locale data
    typedef struct {
        char* decimal_point;
        char* thousands_sep;
        char* grouping;
        char* int_curr_symbol;
        char* currency_symbol;
        char* mon_decimal_point;
        char* mon_thousands_sep;
        char* mon_grouping;
        char* positive_sign;
        char* negative_sign;
        char int_frac_digits;
        char frac_digits;
        char p_cs_precedes;
        char p_sep_by_space;
        char n_cs_precedes;
        char n_sep_by_space;
        char p_sign_posn;
        char n_sign_posn;
    } lconv;
    lconv data;
    data.decimal_point = (char*)upload_system_string(".", "lconv.decimal_point");
    data.thousands_sep = (char*)upload_system_string("", "lconv.thousands_sep");
    data.grouping = (char*)upload_system_string("", "lconv.grouping");
    data.int_curr_symbol = (char*)upload_system_string("", "lconv.int_curr_symbol");
    data.currency_symbol = (char*)upload_system_string("", "lconv.currency_symbol");
    data.mon_decimal_point = (char*)upload_system_string("", "lconv.mon_decimal_point");
    data.mon_thousands_sep = (char*)upload_system_string("", "lconv.mon_thousands_sep");
    data.mon_grouping = (char*)upload_system_string("", "lconv.mon_grouping");
    data.positive_sign = (char*)upload_system_string("", "lconv.positive_sign");
    data.negative_sign = (char*)upload_system_string("", "lconv.negative_sign");
    data.int_frac_digits = 127;
    data.frac_digits = 127;
    data.p_cs_precedes = 127;
    data.p_sep_by_space = 127;
    data.n_cs_precedes = 127;
    data.n_sep_by_space = 127;
    data.p_sign_posn = 127;
    data.n_sign_posn = 127;

    lconv_slot = section_stack->get_annotated(sizeof(lconv), "lconv struct", Annotation::Type::OBJECT);
    computer->memory.write_memory(lconv_slot, &data);
}


ulong OS::Windows::upload_system_string(const std::string& str, const char* description)
{
    auto str_slot = section_stack->get_annotated(str.size() + 1, description, Annotation::Type::OBJECT);
    computer->memory.write_str(str_slot.start_address, str);
    return str_slot.start_address;
}

ulong OS::Windows::get_return_64()
{
    return func_call.get_return_64();
}

void OS::Windows::set_param1_32(uint p)
{
    func_call.set_param1_32(p);
}

void OS::Windows::set_param1_64(ulong p)
{
    func_call.set_param1_64(p);
}

void OS::Windows::set_param2_32(uint p)
{
    func_call.set_param2_32(p);
}

void OS::Windows::set_param2_64(ulong p)
{
    func_call.set_param2_64(p);
}

void OS::Windows::set_param3_32(uint p)
{
    func_call.set_param3_32(p);
}

void OS::Windows::set_param3_64(ulong p)
{
    func_call.set_param3_64(p);
}

void OS::Windows::set_param1_double(double p)
{
    func_call.set_param1_double(p);
}

void OS::Windows::set_return_64(ulong r)
{
    func_call.set_return_64(r);
}

ulong OS::Windows::get_param1_64()
{
    return func_call.get_param1_64();
}

ulong OS::Windows::get_param2_64()
{
    return func_call.get_param2_64();
}
