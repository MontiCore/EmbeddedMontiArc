/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "os_windows/windows_system_calls.h"
#include <cmath>

//#include <Windows.h>

using namespace std;

void initterm_core(Computer& c);

void WindowsSystemCalls::add_windows_calls( SystemCalls &sys_calls, OS::Windows &windows ) {
    const char *reason = "windows";

    sys_calls.add_syscall(SysCall("LoadLibraryExW", "KERNEL32.DLL", [](auto& computer) {
        auto name_addr = computer.func_call_windows.get_param1_64();
        auto name_str = computer.memory.read_wstr_as_str(name_addr);
        string name = (char*)name_str;
        if (name.compare("api-ms-win-core-fibers-l1-1-1") == 0 ||
            name.compare("api-ms-win-core-synch-l1-2-0") == 0) {
            computer.func_call_windows.set_return_64(0);
            return true;
        }
        ulong handle;
        auto sym = computer.symbols.get_symbol(name);
        if (sym.type == Symbols::Symbol::Type::HANDLE)
            handle = sym.addr;
        else {
            handle = computer.handles.add_handle((char*)name_str);
            computer.symbols.add_symbol(name, Symbols::Symbol::Type::HANDLE, handle);
        }
        computer.func_call_windows.set_return_64(handle);
        return true;
    }), reason);


    sys_calls.add_syscall( SysCall( "GetProcAddress", "KERNEL32.DLL", [](Computer& computer) {
        auto mod = computer.func_call_windows.get_param1_64();
        auto sec_ptr = computer.memory.get_section(mod);
        if (sec_ptr == nullptr || sec_ptr != computer.handles.section) {
            Log::err.log_tag("Invalid Module HANDLE (%s)", to_hex(mod).c_str());
            return false;
        }
        auto note_ptr = computer.handles.section->annotations.get_annotation(mod);
        if (note_ptr == nullptr) {
            Log::err.log_tag("Module HANDLE does not exists (%s)", to_hex(mod).c_str());
            return false;
        }
        auto name_addr = computer.func_call_windows.get_param2_64();
        auto name_str = computer.memory.read_wstr_as_str(name_addr);
        string name = (char*)name_str;

        ulong call_addr;
        auto sym = computer.symbols.get_symbol(name);
        if (sym.type == Symbols::Symbol::Type::SYSCALL)
            call_addr = sym.addr;
        else
            call_addr = computer.sys_calls.add_syscall(SysCall(name, "", nullptr), "get_proc_address resolve");
        computer.func_call_windows.set_return_64(call_addr);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "GetProcessHeap", "KERNEL32.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_64(computer.heap.heap_handle);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "HeapAlloc", "KERNEL32.DLL", [](Computer& computer) {
        auto heap_handle = computer.func_call_windows.get_param1_64();
        if (computer.heap.heap_handle != heap_handle) {
            Log::err.log_tag("Trying to alloc on non-existing heap");
            return false;
        }
        auto flags = computer.func_call_windows.get_param2_64();
        auto byte_count = computer.func_call_windows.get_param3_64();
        uint64_t addr;
        if (computer.heap.alloc(byte_count, addr))
            computer.func_call_windows.set_return_64(addr);
        else
            computer.func_call_windows.set_return_64(0);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "HeapFree", "KERNEL32.DLL", [](Computer& computer) {
        auto heap_handle = computer.func_call_windows.get_param1_64();
        if (computer.heap.heap_handle != heap_handle) {
            Log::err.log_tag("Trying to alloc on non-existing heap");
            return false;
        }
        auto flags = computer.func_call_windows.get_param2_64();
        auto memory = computer.func_call_windows.get_param3_64();
        if (computer.heap.free(memory))
            computer.func_call_windows.set_return_64(1);
        else
            computer.func_call_windows.set_return_64(0);
        return true;
    }), reason );


    /*sys_calls.add_syscall( SysCall( "GetCommandLineA", "KERNEL32.DLL", []( Computer &computer ) {
        computer.func_call_windows.set_return( computer.cmd_line_str );
        return true;
    } ), reason );
    sys_calls.add_syscall( SysCall( "GetCommandLineW", "KERNEL32.DLL", []( Computer &computer ) {
        computer.func_call_windows.set_return( computer.cmd_line_wstr );
        return true;
    } ), reason );*/


    sys_calls.add_syscall( SysCall( "GetModuleHandleW", "KERNEL32.DLL", [](Computer& computer) {
        auto name_addr = computer.func_call_windows.get_param1_64();
        auto name_str = computer.memory.read_wstr_as_str(name_addr);
        string name = (char*)name_str;
        auto sym = computer.symbols.get_symbol(name);
        ulong res = 0;
        if (sym.type == Symbols::Symbol::Type::HANDLE)
            res = sym.addr;
        computer.func_call_windows.set_return_64(res);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "GetCurrentProcessId", "KERNEL32.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_64(0x25);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "GetSystemTimeAsFileTime", "KERNEL32.DLL", [](Computer& computer) {
        auto addr = computer.func_call_windows.get_param1_64();
        computer.memory.write_long_word(addr, 0x123);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "QueryPerformanceCounter", "KERNEL32.DLL", [](Computer& computer) {
        auto addr = computer.func_call_windows.get_param1_64();
        computer.memory.write_long_word(addr, 0x456);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "GetCurrentThreadId", "KERNEL32.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_64(0x29);
        return true;
    }), reason );


    sys_calls.add_syscall(SysCall("MultiByteToWideChar", "KERNEL32.DLL", [](Computer& computer) {
        //#include <Windows.h>
/*
int MultiByteToWideChar(
  UINT                              CodePage,
  DWORD                             dwFlags,
  _In_NLS_string_(cbMultiByte)LPCCH lpMultiByteStr,
  int                               cbMultiByte,
  LPWSTR                            lpWideCharStr,
  int                               cchWideChar
);

ERROR_INSUFFICIENT_BUFFER = 122L
*/
        auto code_page = computer.func_call_windows.get_param1_32();
        auto dwFlags = computer.func_call_windows.get_param2_32();
        auto lpMultiByteStr = computer.func_call_windows.get_param3_64();
        auto cbMultiByte = computer.func_call_windows.get_param4_32();
        auto lpWideCharStr = computer.func_call_windows.get_param_after_4_64(5);
        auto cchWideChar = computer.func_call_windows.get_param_after_4_32(6);

        throw_assert(code_page == 65001, "Unsupported CodePage");

        char* source;
        if (cbMultiByte < 0) {
            source = computer.memory.read_str(lpMultiByteStr);
        }
        else {
            source = (char*)computer.memory.read_memory(lpMultiByteStr, cbMultiByte);
            source[cbMultiByte] = '\0';
        }

        /*if (computer.debug.syscalls()) {
            Log::sys.log_tag("MultiByteToWideChar(%u, %u, %s, %u, %s, %u)", code_page, dwFlags, to_hex(lpMultiByteStr).c_str(), cbMultiByte, to_hex(lpWideCharStr).c_str(), cchWideChar);
            Log::sys.log_tag("    with lpMultiByteStr=\"%s\"", source);
        }*/
        int res = 0;
        if (cchWideChar > 0) {
            std::vector<wchar_t> buff(cchWideChar);
            res = std::mbtowc(buff.data(), source, cbMultiByte);
            //res = MultiByteToWideChar(code_page, dwFlags, source, cbMultiByte, buff.data(), cchWideChar);
            //throw_assert(res == 0, "MultiByteToWideChar error");
            if (res == 0) {
                //throw_lasterr("MultiByteToWideChar");
            }
            computer.memory.write_memory(lpWideCharStr, cchWideChar * 2, buff.data());
        }
        else {
            //res = MultiByteToWideChar(code_page, dwFlags, source, cbMultiByte, nullptr, 0);
            Log::err.log_tag("Unsupported MultiByteToWideChar case.");
        }

        computer.func_call_windows.set_return_64(res);
        return true;
    }), reason);

    sys_calls.add_syscall(SysCall("WideCharToMultiByte", "KERNEL32.DLL", [](Computer& computer) {
        //#include <Windows.h>
/*
int WideCharToMultiByte(
  UINT                               CodePage,
  DWORD                              dwFlags,
  _In_NLS_string_(cchWideChar)LPCWCH lpWideCharStr,
  int                                cchWideChar,
  LPSTR                              lpMultiByteStr,
  int                                cbMultiByte,
  LPCCH                              lpDefaultChar,
  LPBOOL                             lpUsedDefaultChar
);
*/
        auto code_page = computer.func_call_windows.get_param1_32();
        auto dwFlags = computer.func_call_windows.get_param2_32();
        auto lpWideCharStr = computer.func_call_windows.get_param3_64();
        auto cchWideChar = computer.func_call_windows.get_param4_32();
        auto lpMultiByteStr = computer.func_call_windows.get_param_after_4_64(5);
        auto cbMultiByte = computer.func_call_windows.get_param_after_4_32(6);
        auto lpDefaultChar_ptr = computer.func_call_windows.get_param_after_4_64(7);
        auto lpUsedDefaultChar_ptr = computer.func_call_windows.get_param_after_4_64(8);

        auto input_str = computer.memory.read_wstr(lpWideCharStr);

        if (computer.debug.unsupported_syscalls()) {
            Log::sys.log_tag("MultiByteToWideChar(%u, %u, %s, %u, %s, %u, %s, %s)", 
                code_page, dwFlags, 
                to_hex(lpMultiByteStr).c_str(), cbMultiByte, 
                to_hex(lpWideCharStr).c_str(), cchWideChar,
                to_hex(lpDefaultChar_ptr).c_str(), to_hex(lpUsedDefaultChar_ptr).c_str());
            Log::sys.log_tag("    with cchWideChar=\"%ls\"", input_str);
        }
        computer.func_call_windows.set_return_64(0);
        return true;
    }), reason);


    sys_calls.add_syscall(SysCall("VirtualQuery", "KERNEL32.DLL", [](Computer& computer) {
        typedef struct _MEMORY_BASIC_INFORMATION {
            ulong  BaseAddress;
            ulong  AllocationBase;
            uint  AllocationProtect;
            ulong RegionSize;
            uint  State;
            uint  Protect;
            uint  Type;
        } MEMORY_BASIC_INFORMATION, * PMEMORY_BASIC_INFORMATION;
        MEMORY_BASIC_INFORMATION mem_info;

        auto lpaddr = computer.func_call_windows.get_param1_64();
        auto lpbuffer = computer.func_call_windows.get_param2_64();
        auto size = computer.func_call_windows.get_param3_64();
        if (size < sizeof(MEMORY_BASIC_INFORMATION))
            Log::err.log_tag("VirtualQuery with size < sizeof(MEMORY_BASIC_INFORMATION)");
        auto sec_ptr = computer.memory.get_section(lpaddr);
        if (sec_ptr == nullptr) {
            Log::err.log_tag("VirtualQuery outside pages");
            return false;
        }
        //printf( "VirtualQuery(%016" PRIX64 ")", lpaddr );
        auto& sec = *sec_ptr;
        mem_info.BaseAddress = sec.address_range.start_address;
        mem_info.AllocationBase = sec.address_range.start_address;
        mem_info.AllocationProtect = 0;
        mem_info.RegionSize = 0;
        mem_info.State = 0;
        mem_info.Protect = 0;
        mem_info.Type = 0;
        computer.memory.write_memory(lpbuffer, size, (uchar*)&mem_info);
        computer.func_call_windows.set_return_64(size);
        return true;
    }), reason);


    sys_calls.add_syscall(SysCall("VirtualProtect", "KERNEL32.DLL", [](Computer& computer) {
        auto lpaddr = computer.func_call_windows.get_param1_64();
        auto size = computer.func_call_windows.get_param2_64();
        auto protect = computer.func_call_windows.get_param3_64();
        //printf( "VirtualProtect(%016" PRIX64 ", %" PRIX64", %" PRIX64")", lpaddr, size, protect );
        computer.func_call_windows.set_return_64(1);
        return true;
    }), reason);
    sys_calls.add_syscall(SysCall("EnterCriticalSection", "KERNEL32.DLL", [](Computer& c) { return true; }), reason);
    sys_calls.add_syscall(SysCall("LeaveCriticalSection", "KERNEL32.DLL", [](Computer& c) { return true; }), reason);
    sys_calls.add_syscall(SysCall("InitializeCriticalSection", "KERNEL32.DLL", [](Computer& c) { return true; }), reason);
    

    sys_calls.add_syscall( SysCall( "strlen", "MSVCRT.DLL", [](Computer& computer) {
        auto addr = computer.func_call_windows.get_param1_64();
        auto name_str = computer.memory.read_str(addr);
        string name = (char*)name_str;
        uint size = (uint)name.size();
        //Log::sys << Log::tag << "strlen(\"" << name << "\"): " << to_string( size ) << "\n";
        computer.func_call_windows.set_return_64(size);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "strncmp", "MSVCRT.DLL", [](Computer& computer) {
        auto addr1 = computer.func_call_windows.get_param1_64();
        auto addr2 = computer.func_call_windows.get_param2_64();
        auto count = computer.func_call_windows.get_param3_64();
        string str1 = (char*)computer.memory.read_str(addr1);
        string str2 = (char*)computer.memory.read_str(addr2);
        sint res = ::strncmp(str1.c_str(), str2.c_str(), count);
        /*Log::sys << Log::tag << "strncmp(\"" << str1 << "\",\"" << str2 << "\"," + to_string( count ) << "\"): " << to_string(
                     res ) << "\n";*/
        computer.func_call_windows.set_return_64(res);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "__iob_func", "MSVCRT.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_64(computer.io_slot.start_address);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "abort", "MSVCRT.DLL", [](Computer& computer) {
        computer.exit_emulation();
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "fwrite", "MSVCRT.DLL", [](Computer& computer) {
        //size_t fwrite ( const void * ptr, size_t size, size_t count, FILE * stream );
        auto data = computer.func_call_windows.get_param1_64();
        auto size = computer.func_call_windows.get_param2_64();
        auto count = computer.func_call_windows.get_param3_64();
        auto stream = computer.func_call_windows.get_param4_64();
        string str1 = (char*)computer.memory.read_str(data);
        //printf( "fwrite(\"%s\", %d, %d, %016" PRIX64 ")", str1.c_str(), size, count, stream );
        computer.func_call_windows.set_return_64(size * count);
        return false;
    }), reason );


    sys_calls.add_syscall( SysCall( "malloc", "MSVCRT.DLL", [](Computer& computer) {
        auto byte_count = computer.func_call_windows.get_param1_64();
        //cout << "malloc(" << byte_count << ")" << endl;
        uint64_t addr;
        if (computer.heap.alloc(byte_count, addr))
            computer.func_call_windows.set_return_64(addr);
        else
            computer.func_call_windows.set_return_64(0);
        return true;
    }), reason );

    sys_calls.add_syscall(SysCall("calloc", "MSVCRT.DLL", [](Computer& computer) {
        auto byte_count = computer.func_call_windows.get_param1_64();
        //cout << "calloc(" << byte_count << ")" << endl;
        uint64_t addr;
        if (computer.heap.alloc(byte_count, addr)) {
            computer.func_call_windows.set_return_64(addr);
            for (int i = 0; i < 8; ++i) {
                computer.memory.buffer[i] = 0;
            }
            while (byte_count >= 8) {
                computer.memory.write_memory_buffer(addr, 8);
                addr += 8;
                byte_count -= 8;
            }
            if (byte_count > 0) {
                computer.memory.write_memory_buffer(addr, byte_count);
            }
        }
        else
            computer.func_call_windows.set_return_64(0);
        return true;
    }), reason);


    sys_calls.add_syscall( SysCall( "memcpy", "MSVCRT.DLL", [](Computer& computer) {
        auto target_pointer = computer.func_call_windows.get_param1_64();
        auto source_pointer = computer.func_call_windows.get_param2_64();
        auto size = computer.func_call_windows.get_param3_64();
        if (computer.debug.syscalls())
            Log::sys.log_tag("memcpy(%s, %s, %llu)", to_hex(target_pointer).c_str(), to_hex(source_pointer).c_str(), size);
        auto r = computer.memory.read_memory(source_pointer, size);
        computer.memory.write_memory(target_pointer, size, r);
        return true;
    }), reason );


    sys_calls.add_syscall( SysCall( "acos", "MSVCRT.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_double(::acos(computer.func_call_windows.get_param1_double()));
        return true;
    }), reason );


    sys_calls.add_syscall(SysCall("localeconv", "MSVCRT.DLL", [](Computer& computer) {
        computer.func_call_windows.set_return_64(((OS::Windows*)computer.os.get())->lconv_slot.start_address);
        return true;
    }), reason);

    sys_calls.add_syscall(SysCall("strcmp", "MSVCRT.DLL", [](Computer& computer) {
        auto str1_ptr = computer.func_call_windows.get_param1_64();
        auto str2_ptr = computer.func_call_windows.get_param2_64();

        std::string str1 = computer.memory.read_str(str1_ptr);
        auto str2 = computer.memory.read_str(str2_ptr);

        auto res = strcmp(str1.c_str(), str2);
        computer.func_call_windows.set_return_64(res);
        return true;
    }), reason);

    sys_calls.add_syscall(SysCall("memset", "MSVCRT.DLL", [](Computer& computer) {
        // void * memset ( void * ptr, int value, size_t num );
        auto ptr = computer.func_call_windows.get_param1_64();
        auto value = computer.func_call_windows.get_param2_32();
        auto num = computer.func_call_windows.get_param3_64();
        std::vector<uint8_t> data(num, (uint8_t)value);
        computer.memory.write_memory(ptr, num, data.data());
        computer.func_call_windows.set_return_64(ptr);
        return true;
    }), reason);


    sys_calls.add_syscall(SysCall("___lc_codepage_func", "MSVCRT.DLL", [](Computer& computer) {
        //computer.func_call_windows.set_return_64(1252); // ANSI Latin 1; Western European (Windows)
        computer.func_call_windows.set_return_64(65001); // Unicode (UTF-8)

        return true;
    }), reason);
    sys_calls.add_syscall(SysCall("_lock", "MSVCRT.DLL", [](Computer& c) { return true; }), reason);
    sys_calls.add_syscall(SysCall("_unlock", "MSVCRT.DLL", [](Computer& c) { return true; }), reason);

    

    sys_calls.add_syscall(SysCall("_initterm", "MSVCRT.DLL", [](Computer& c) { 
        c.walk_table_pos = c.func_call_windows.get_param1_64();
        c.walk_table_end = c.func_call_windows.get_param2_64();

        initterm_core(c);

        return true; 
    }), reason);

    sys_calls.after_initterm = sys_calls.add_syscall(SysCall("__AFTER___initterm", "MSVCRT.DLL", [](Computer& c) {
        c.call_inside_after();

        initterm_core(c);

        return true;
    }), reason);


    sys_calls.add_syscall(SysCall("pthread_mutex_init", "LIBWINPTHREAD-1.DLL", [](Computer& c) { c.func_call_windows.set_return_64(0); return true; }), reason);


    sys_calls.add_syscall(SysCall("pthread_once", "LIBWINPTHREAD-1.DLL", [](Computer& c) {
#define PTHREAD_ONCE_INIT 0
        /*
        int pthread_once(
            pthread_once_t *once_control,
            void (*init_routine)(void)
        );
        pthread_once_t once_control = PTHREAD_ONCE_INIT; 
        */
        auto once_control_ptr = c.func_call_windows.get_param1_64();
        auto callback_addr = c.func_call_windows.get_param2_64();

        uint32_t once_control = *((uint32_t*)c.memory.read_memory(once_control_ptr, 4));
        if (once_control == PTHREAD_ONCE_INIT) {
            once_control = 1;
            c.memory.write_memory(once_control_ptr, 4, &once_control);
            c.call_inside(callback_addr, c.sys_calls.after_pthread_once_function);
        }

        return true; 
#undef PTHREAD_ONCE_INIT
    }), reason);

    sys_calls.after_pthread_once_function = sys_calls.add_syscall(SysCall("__AFTER__pthread_once", "LIBWINPTHREAD-1.DLL", [](Computer& c) {
        c.call_inside_after();
        c.func_call_windows.set_return_64(0);
        return true;
    }), reason);

    sys_calls.add_syscall(SysCall("pthread_key_create", "LIBWINPTHREAD-1.DLL", [](Computer& c) {
        auto ptr = c.func_call_windows.get_param1_64();
        uint32_t v = 0x42424242;
        c.memory.write_memory(ptr, 4, &v);
        c.func_call_windows.set_return_64(0); 
        return true; 
    }), reason);
}

void initterm_core(Computer &c) {
    auto target_func_addr = *((ulong*)c.memory.read_memory(c.walk_table_pos, 8));
    if (c.walk_table_pos > c.walk_table_end) return;
    c.walk_table_pos += 8;
    while (target_func_addr == 0) {
        if (c.walk_table_pos > c.walk_table_end) return;
        target_func_addr = *((ulong*)c.memory.read_memory(c.walk_table_pos, 8));
        c.walk_table_pos += 8;
    }
    c.call_inside(target_func_addr, c.sys_calls.after_initterm);
}


//#include <Windows.h>
