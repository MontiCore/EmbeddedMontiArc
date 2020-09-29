/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "flags.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

extern std::ofstream ofs;

Flag file_characteristics_flags[] = {
    {0x0001, "IMAGE_FILE_RELOCS_STRIPPED"},
    {0x0002, "IMAGE_FILE_EXECUTABLE_IMAGE"},
    {0x0004, "IMAGE_FILE_LINE_NUMS_STRIPPED"},
    {0x0008, "IMAGE_FILE_LOCAL_SYMS_STRIPPED"},
    {0x0010, "IMAGE_FILE_AGGRESIVE_WS_TRIM"},
    {0x0020, "IMAGE_FILE_LARGE_ADDRESS_AWARE"},
    {0x0080, "IMAGE_FILE_BYTES_REVERSED_LO"},
    {0x0100, "IMAGE_FILE_32BIT_MACHINE"},
    {0x0200, "IMAGE_FILE_DEBUG_STRIPPED"},
    {0x0400, "IMAGE_FILE_REMOVABLE_RUN_FROM_SWAP"},
    {0x0800, "IMAGE_FILE_NET_RUN_FROM_SWAP"},
    {0x1000, "IMAGE_FILE_SYSTEM"},
    {0x2000, "IMAGE_FILE_DLL"},
    {0x4000, "IMAGE_FILE_UP_SYSTEM_ONLY"},
    {0x8000, "IMAGE_FILE_BYTES_REVERSED_HI"},
    {0x0, ""}
};

Flag machine_values[] = {
    {0x14c, "Intel 386"},
    {0x8664, "x64"},
    {0x162, "MIPS R3000"},
    {0x168, "MIPS R10000"},
    {0x169, "MIPS little endian WCI v2"},
    {0x183, "old Alpha AXP"},
    {0x184, "Alpha AXP"},
    {0x1a2, "Hitachi SH3"},
    {0x1a3, "Hitachi SH3 DSP"},
    {0x1a6, "Hitachi SH4"},
    {0x1a8, "Hitachi SH5"},
    {0x1c0, "ARM little endian"},
    {0x1c2, "Thumb"},
    {0x1c4, "ARMv7"},
    {0x1d3, "Matsushita AM33"},
    {0x1f0, "PowerPC little endian"},
    {0x1f1, "PowerPC with floating point support"},
    {0x200, "Intel IA64"},
    {0x266, "MIPS16"},
    {0x268, "Motorola 68000 series"},
    {0x284, "Alpha AXP 64-bit"},
    {0x366, "MIPS with FPU"},
    {0x466, "MIPS16 with FPU"},
    {0xebc, "EFI Byte Code"},
    {0x8664, "AMD AMD64"},
    {0x9041, "Mitsubishi M32R little endian"},
    {0xaa64, "ARM64 little endian"},
    {0xc0ee, "clr pure MSIL"},
    {0x0, ""},
};

Flag signature_values[] = {
    {0x10b, "IMAGE_NT_OPTIONAL_HDR32_MAGIC"},
    {0x20b, "IMAGE_NT_OPTIONAL_HDR64_MAGIC"},
    {0x107, "IMAGE_ROM_OPTIONAL_HDR_MAGIC"},
};

Flag subsystem_values[] = {
    {0, "IMAGE_SUBSYSTEM_UNKNOWN"},
    {1, "IMAGE_SUBSYSTEM_NATIVE"},
    {2, "IMAGE_SUBSYSTEM_WINDOWS_GUI"},
    {3, "IMAGE_SUBSYSTEM_WINDOWS_CUI"},
    {5, "IMAGE_SUBSYSTEM_OS2_CUI"},
    {7, "IMAGE_SUBSYSTEM_POSIX_CUI"},
    {9, "IMAGE_SUBSYSTEM_WINDOWS_CE_GUI"},
    {10, "IMAGE_SUBSYSTEM_EFI_APPLICATION"},
    {11, "IMAGE_SUBSYSTEM_EFI_BOOT_SERVICE_DRIVER"},
    {12, "IMAGE_SUBSYSTEM_EFI_RUNTIME_DRIVER"},
    {13, "IMAGE_SUBSYSTEM_EFI_ROM"},
    {14, "IMAGE_SUBSYSTEM_XBOX"},
    {16, "IMAGE_SUBSYSTEM_WINDOWS_BOOT_APPLICATION"},
    {0, ""},
};

Flag dll_flags[] = {
    {0x0001, "0x0001"},
    {0x0002, "0x0002"},
    {0x0004, "0x0004"},
    {0x0008, "0x0008"},
    {0x0040, "IMAGE_DLLCHARACTERISTICS_DYNAMIC_BASE"},
    {0x0080, "IMAGE_DLLCHARACTERISTICS_FORCE_INTEGRITY"},
    {0x0100, "IMAGE_DLLCHARACTERISTICS_NX_COMPAT"},
    {0x0200, "IMAGE_DLLCHARACTERISTICS_NO_ISOLATION"},
    {0x0400, "IMAGE_DLLCHARACTERISTICS_NO_SEH"},
    {0x0800, "IMAGE_DLLCHARACTERISTICS_NO_BIND"},
    {0x1000, "IMAGE_DLLCHARACTERISTICS_APPCONTAINER"},
    {0x2000, "IMAGE_DLLCHARACTERISTICS_WDM_DRIVER"},
    {0x4000, "0x4000"},
    {0x8000, "IMAGE_DLLCHARACTERISTICS_TERMINAL_SERVER_AWARE"},
    {0x0, ""},
};

Flag data_directory_values[] = {
    {0, "IMAGE_DIRECTORY_ENTRY_EXPORT"},
    {1, "IMAGE_DIRECTORY_ENTRY_IMPORT"},
    {2, "IMAGE_DIRECTORY_ENTRY_RESOURCE"},
    {3, "IMAGE_DIRECTORY_ENTRY_EXCEPTION"},
    {4, "IMAGE_DIRECTORY_ENTRY_SECURITY"},
    {5, "IMAGE_DIRECTORY_ENTRY_BASERELOC"},
    {6, "IMAGE_DIRECTORY_ENTRY_DEBUG"},
    {7, "IMAGE_DIRECTORY_ENTRY_ARCHITECTURE"},
    {8, "IMAGE_DIRECTORY_ENTRY_GLOBALPTR"},
    {9, "IMAGE_DIRECTORY_ENTRY_TLS"},
    {10, "IMAGE_DIRECTORY_ENTRY_LOAD_CONFIG"},
    {11, "IMAGE_DIRECTORY_ENTRY_BOUND_IMPORT"},
    {12, "IMAGE_DIRECTORY_ENTRY_IAT"},
    {13, "IMAGE_DIRECTORY_ENTRY_DELAY_IMPORT"},
    {14, "IMAGE_DIRECTORY_ENTRY_COM_DESCRIPTOR"},
    {15, "15"},
    {0, ""},
};

Flag section_characteristics_flags[] = {
    {0x00000008, "IMAGE_SCN_TYPE_NO_PAD"},
    {0x00000020, "IMAGE_SCN_CNT_CODE"},
    {0x00000040, "IMAGE_SCN_CNT_INITIALIZED_DATA"},
    {0x00000080, "IMAGE_SCN_CNT_UNINITIALIZED_DATA"},
    {0x00000100, "IMAGE_SCN_LNK_OTHER"},
    {0x00000200, "IMAGE_SCN_LNK_INFO"},
    {0x00000800, "IMAGE_SCN_LNK_REMOVE"},
    {0x00001000, "IMAGE_SCN_LNK_COMDAT"},
    {0x00004000, "IMAGE_SCN_NO_DEFER_SPEC_EXC"},
    {0x00008000, "IMAGE_SCN_GPREL"},
    {0x00020000, "IMAGE_SCN_MEM_PURGEABLE"},
    {0x00040000, "IMAGE_SCN_MEM_LOCKED"},
    {0x00080000, "IMAGE_SCN_MEM_PRELOAD"},
    {0x01000000, "IMAGE_SCN_LNK_NRELOC_OVFL"},
    {0x02000000, "IMAGE_SCN_MEM_DISCARDABLE"},
    {0x04000000, "IMAGE_SCN_MEM_NOT_CACHED"},
    {0x08000000, "IMAGE_SCN_MEM_NOT_PAGED"},
    {0x10000000, "IMAGE_SCN_MEM_SHARED"},
    {0x20000000, "IMAGE_SCN_MEM_EXECUTE"},
    {0x40000000, "IMAGE_SCN_MEM_READ"},
    {0x80000000, "IMAGE_SCN_MEM_WRITE"},
    {0x0, ""},
};

Flag section_align_values[] = {
    {0x00100000, "IMAGE_SCN_ALIGN_1BYTES"},
    {0x00200000, "IMAGE_SCN_ALIGN_2BYTES"},
    {0x00300000, "IMAGE_SCN_ALIGN_4BYTES"},
    {0x00400000, "IMAGE_SCN_ALIGN_8BYTES"},
    {0x00500000, "IMAGE_SCN_ALIGN_16BYTES"},
    {0x00600000, "IMAGE_SCN_ALIGN_32BYTES"},
    {0x00700000, "IMAGE_SCN_ALIGN_64BYTES"},
    {0x00800000, "IMAGE_SCN_ALIGN_128BYTES"},
    {0x00900000, "IMAGE_SCN_ALIGN_256BYTES"},
    {0x00A00000, "IMAGE_SCN_ALIGN_512BYTES"},
    {0x00B00000, "IMAGE_SCN_ALIGN_1024BYTES"},
    {0x00C00000, "IMAGE_SCN_ALIGN_2048BYTES"},
    {0x00D00000, "IMAGE_SCN_ALIGN_4096BYTES"},
    {0x00E00000, "IMAGE_SCN_ALIGN_8192BYTES"},
    {0x0, ""},
};

uint64_t section_align_mask = 0x00F00000;

void print_flags_bit( Flag *flags, uint64_t value ) {
    int i = 0;
    int c = 0;
    do {
        if ( value & flags[i].bit ) {
            if ( c > 0 )
                ofs << ", " << flags[i].name;
                
            else
                ofs << flags[i].name;
            ++c;
        }
        i++;
    } while ( flags[i].bit != 0 );
}

void print_flags_value( Flag *flags, uint64_t value ) {
    int i = 0;
    int c = 0;
    do {
        if ( value == flags[i].bit ) {
            if ( c > 0 )
                ofs << ", " << flags[i].name;
                
            else
                ofs << flags[i].name;
            ++c;
        }
        i++;
    } while ( flags[i].bit != 0 );
}
