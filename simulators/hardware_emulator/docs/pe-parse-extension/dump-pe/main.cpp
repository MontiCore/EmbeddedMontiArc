/**
 * (c) https://github.com/MontiCore/monticore
 */
/*
The MIT License (MIT)

Copyright (c) 2013 Andrew Ruef

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <parser-library/parse.h>
#include "flags.h"

std::ofstream ofs;

using namespace peparse;

int printExps( void *N, VA funcAddr, std::string &mod, std::string &func ) {
    static_cast<void>( N );
    
    auto address = static_cast<std::uint64_t>( funcAddr );
    
    ofs << "EXP: ";
    ofs << mod;
    ofs << "!";
    ofs << func;
    ofs << ": 0x";
    ofs << std::hex << address;
    ofs << "\n";
    return 0;
}

int printImports( void *N,
                  VA impAddr,
                  const std::string &modName,
                  const std::string &symName ) {
    static_cast<void>( N );
    
    auto address = static_cast<std::uint32_t>( impAddr );
    
    ofs << "0x" << std::hex << address << " " << modName << "!" << symName;
    ofs << "\n";
    return 0;
}

int printRelocs( void *N, VA relocAddr, reloc_type type ) {
    static_cast<void>( N );
    
    ofs << "TYPE: ";
    switch ( type ) {
        case ABSOLUTE:
            ofs << "ABSOLUTE";
            break;
        case HIGH:
            ofs << "HIGH";
            break;
        case LOW:
            ofs << "LOW";
            break;
        case HIGHLOW:
            ofs << "HIGHLOW";
            break;
        case HIGHADJ:
            ofs << "HIGHADJ";
            break;
        case MIPS_JMPADDR:
            ofs << "MIPS_JMPADDR";
            break;
        case MIPS_JMPADDR16:
            ofs << "MIPS_JMPADD16";
            break;
        case DIR64:
            ofs << "DIR64";
            break;
        default:
            ofs << "UNKNOWN";
            break;
    }
    
    ofs << " VA: 0x" << std::hex << relocAddr << "\n";
    
    return 0;
}

int printSymbols( void *N,
                  std::string &strName,
                  uint32_t &value,
                  int16_t &sectionNumber,
                  uint16_t &type,
                  uint8_t &storageClass,
                  uint8_t &numberOfAuxSymbols ) {
    static_cast<void>( N );
    
    ofs << "Symbol Name: " << strName << "\n";
    ofs << "Symbol Value: 0x" << std::hex << value << "\n";
    
    ofs << "Symbol Section Number: ";
    switch ( sectionNumber ) {
        case IMAGE_SYM_UNDEFINED:
            ofs << "UNDEFINED";
            break;
        case IMAGE_SYM_ABSOLUTE:
            ofs << "ABSOLUTE";
            break;
        case IMAGE_SYM_DEBUG:
            ofs << "DEBUG";
            break;
        default:
            ofs << sectionNumber;
            break;
    }
    ofs << "\n";
    
    ofs << "Symbol Type: ";
    switch ( type ) {
        case IMAGE_SYM_TYPE_NULL:
            ofs << "NULL";
            break;
        case IMAGE_SYM_TYPE_VOID:
            ofs << "VOID";
            break;
        case IMAGE_SYM_TYPE_CHAR:
            ofs << "CHAR";
            break;
        case IMAGE_SYM_TYPE_SHORT:
            ofs << "SHORT";
            break;
        case IMAGE_SYM_TYPE_INT:
            ofs << "INT";
            break;
        case IMAGE_SYM_TYPE_LONG:
            ofs << "LONG";
            break;
        case IMAGE_SYM_TYPE_FLOAT:
            ofs << "FLOAT";
            break;
        case IMAGE_SYM_TYPE_DOUBLE:
            ofs << "DOUBLE";
            break;
        case IMAGE_SYM_TYPE_STRUCT:
            ofs << "STRUCT";
            break;
        case IMAGE_SYM_TYPE_UNION:
            ofs << "UNION";
            break;
        case IMAGE_SYM_TYPE_ENUM:
            ofs << "ENUM";
            break;
        case IMAGE_SYM_TYPE_MOE:
            ofs << "IMAGE_SYM_TYPE_MOE";
            break;
        case IMAGE_SYM_TYPE_BYTE:
            ofs << "BYTE";
            break;
        case IMAGE_SYM_TYPE_WORD:
            ofs << "WORD";
            break;
        case IMAGE_SYM_TYPE_UINT:
            ofs << "UINT";
            break;
        case IMAGE_SYM_TYPE_DWORD:
            ofs << "DWORD";
            break;
        default:
            ofs << "UNKNOWN";
            break;
    }
    ofs << "\n";
    
    ofs << "Symbol Storage Class: ";
    switch ( storageClass ) {
        case IMAGE_SYM_CLASS_END_OF_FUNCTION:
            ofs << "FUNCTION";
            break;
        case IMAGE_SYM_CLASS_NULL:
            ofs << "NULL";
            break;
        case IMAGE_SYM_CLASS_AUTOMATIC:
            ofs << "AUTOMATIC";
            break;
        case IMAGE_SYM_CLASS_EXTERNAL:
            ofs << "EXTERNAL";
            break;
        case IMAGE_SYM_CLASS_STATIC:
            ofs << "STATIC";
            break;
        case IMAGE_SYM_CLASS_REGISTER:
            ofs << "REGISTER";
            break;
        case IMAGE_SYM_CLASS_EXTERNAL_DEF:
            ofs << "EXTERNAL DEF";
            break;
        case IMAGE_SYM_CLASS_LABEL:
            ofs << "LABEL";
            break;
        case IMAGE_SYM_CLASS_UNDEFINED_LABEL:
            ofs << "UNDEFINED LABEL";
            break;
        case IMAGE_SYM_CLASS_MEMBER_OF_STRUCT:
            ofs << "MEMBER OF STRUCT";
            break;
        default:
            ofs << "UNKNOWN";
            break;
    }
    ofs << "\n";
    
    ofs << "Symbol Number of Aux Symbols: "
        << static_cast<std::uint32_t>( numberOfAuxSymbols ) << "\n";
        
    return 0;
}


void print_number( uint64_t val ) {
    ofs << "0x" << std::hex << val << "/" << std::dec << val;
}

void print_number( char *name, uint64_t val ) {
    ofs << name  << ": ";
    print_number( val );
    ofs << "\n";
}

int printRsrc( void *N, resource r ) {
    static_cast<void>( N );
    
    if ( r.type_str.length() )
        ofs << "Type (string): " << r.type_str << "\n";
    else
        ofs << "Type: 0x" << std::hex << r.type << "\n";
        
    if ( r.name_str.length() )
        ofs << "Name (string): " << r.name_str << "\n";
    else
        ofs << "Name: 0x" << std::hex << r.name << "\n";
        
    if ( r.lang_str.length() )
        ofs << "Lang (string): " << r.lang_str << "\n";
    else
        ofs << "Lang: 0x" << std::hex << r.lang << "\n";
        
    ofs << "Codepage: 0x" << std::hex << r.codepage << "\n";
    ofs << "RVA: " << std::dec << r.RVA << "\n";
    ofs << "Size: " << std::dec << r.size << "\n";
    return 0;
}

int printSecs( void *N,
               VA secBase,
               std::string &secName,
               image_section_header s,
               bounded_buffer *data ) {
    static_cast<void>( N );
    
    ofs << "Sec Name: " << secName << "\n";
    print_number( "Sec Base", secBase );
    if ( data )
        print_number( "Sec Size", data->bufLen );
    else
        ofs << "Sec Size: 0" << "\n";
    ofs << "Header:\n";
    uint8_t name[9];
    for ( int i = 0; i < 8; ++i )
        name[i] = s.Name[i];
    name[8] = 0;
    ofs << "\tName=" << name << "\n";
    print_number( "\tPhysicalAddress/VirtualSize", s.Misc.VirtualSize );
    print_number( "\tVirtualAddress", s.VirtualAddress );
    print_number( "\tSizeOfRawData", s.SizeOfRawData );
    print_number( "\tPointerToRawData", s.PointerToRawData );
    print_number( "\tPointerToRelocations", s.PointerToRelocations );
    print_number( "\tPointerToLinenumbers", s.PointerToLinenumbers );
    print_number( "\tNumberOfRelocations", s.NumberOfRelocations );
    print_number( "\tNumberOfLinenumbers", s.NumberOfLinenumbers );
    print_number( "\tCharacteristics", s.Characteristics );
    ofs << "\t= ";
    print_flags_bit( section_characteristics_flags, s.Characteristics );
    ofs << ", ";
    print_flags_value( section_align_values, s.Characteristics & section_align_mask );
    ofs << '\n';
    return 0;
}


#define DUMP_FIELD(x)     \
    ofs << "" #x << ": "; \
    print_number(static_cast<std::uint64_t>(p->peHeader.nt.x)); \
    ofs << "\n"
#define DUMP_VAL(x) static_cast<std::uint64_t>(p->peHeader.nt.x)



int main( int argc, char *argv[] ) {
    ofs = std::ofstream( "output.txt", std::ofstream::out );
    if ( argc != 2 || ( argc == 2 && std::strcmp( argv[1], "--help" ) == 0 ) ) {
        ofs << "dump-pe utility from Trail of Bits\n";
        ofs << "Repository: https://github.com/trailofbits/pe-parse\n\n";
        ofs << "Usage:\n\tdump-pe /path/to/executable.exe\n";
        return 1;
    }
    
    /*std::string file = "A:\\EmbededMontiArcNew\\unicorn\\SampleUseTest\\x64\\Debug\\AutopilotModel.dll";*/
    parsed_pe *p = ParsePEFromFile( argv[1] );
    // parsed_pe *p = ParsePEFromFile(
    // "A:\\EmbededMontiArcNew\\EMAM-showcase\\dll\\AutopilotAdapter.dll" );
    // parsed_pe *p = ParsePEFromFile( file.c_str() );
    ///*parsed_pe *p = ParsePEFromFile( "A:\\EmbededMontiArcNew\\EMAM-"
    //                                "showcase\\VS17\\AutopilotModel\\x64\\Release\\Au"
    //                                "topilotModel.dll" );*/
    ofs << "File: " << argv[1] << std::endl;
    if ( p != NULL ) {
        // print out some things
        DUMP_FIELD( Signature );
        DUMP_FIELD( FileHeader.Machine );
        ofs << "\t= ";
        print_flags_value( machine_values, DUMP_VAL( FileHeader.Machine ) );
        ofs << '\n';
        DUMP_FIELD( FileHeader.NumberOfSections );
        DUMP_FIELD( FileHeader.TimeDateStamp );
        DUMP_FIELD( FileHeader.PointerToSymbolTable );
        DUMP_FIELD( FileHeader.NumberOfSymbols );
        DUMP_FIELD( FileHeader.SizeOfOptionalHeader );
        DUMP_FIELD( FileHeader.Characteristics );
        ofs << "\t= ";
        print_flags_bit( file_characteristics_flags,
                         DUMP_VAL( FileHeader.Characteristics ) );
        ofs << '\n';
        if ( p->peHeader.nt.OptionalMagic == NT_OPTIONAL_32_MAGIC ) {
            DUMP_FIELD( OptionalHeader.Magic );
            ofs << "\t= ";
            print_flags_value( signature_values,
                               DUMP_VAL( OptionalHeader.Magic ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader.MajorLinkerVersion );
            DUMP_FIELD( OptionalHeader.MinorLinkerVersion );
            DUMP_FIELD( OptionalHeader.SizeOfCode );
            DUMP_FIELD( OptionalHeader.SizeOfInitializedData );
            DUMP_FIELD( OptionalHeader.SizeOfUninitializedData );
            DUMP_FIELD( OptionalHeader.AddressOfEntryPoint );
            DUMP_FIELD( OptionalHeader.BaseOfCode );
            DUMP_FIELD( OptionalHeader.BaseOfData );
            DUMP_FIELD( OptionalHeader.ImageBase );
            DUMP_FIELD( OptionalHeader.SectionAlignment );
            DUMP_FIELD( OptionalHeader.FileAlignment );
            DUMP_FIELD( OptionalHeader.MajorOperatingSystemVersion );
            DUMP_FIELD( OptionalHeader.MinorOperatingSystemVersion );
            DUMP_FIELD( OptionalHeader.MajorImageVersion );
            DUMP_FIELD( OptionalHeader.MinorImageVersion );
            DUMP_FIELD( OptionalHeader.MajorSubsystemVersion );
            DUMP_FIELD( OptionalHeader.MinorSubsystemVersion );
            DUMP_FIELD( OptionalHeader.Win32VersionValue );
            DUMP_FIELD( OptionalHeader.SizeOfImage );
            DUMP_FIELD( OptionalHeader.SizeOfHeaders );
            DUMP_FIELD( OptionalHeader.CheckSum );
            DUMP_FIELD( OptionalHeader.Subsystem );
            ofs << "\t= ";
            print_flags_value( subsystem_values,
                               DUMP_VAL( OptionalHeader.Subsystem ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader.DllCharacteristics );
            ofs << "\t= ";
            print_flags_bit( dll_flags,
                             DUMP_VAL( OptionalHeader.DllCharacteristics ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader.SizeOfStackReserve );
            DUMP_FIELD( OptionalHeader.SizeOfStackCommit );
            DUMP_FIELD( OptionalHeader.SizeOfHeapReserve );
            DUMP_FIELD( OptionalHeader.SizeOfHeapCommit );
            DUMP_FIELD( OptionalHeader.LoaderFlags );
            DUMP_FIELD( OptionalHeader.NumberOfRvaAndSizes );
            auto nb_rva_size = DUMP_VAL( OptionalHeader.NumberOfRvaAndSizes );
            if ( nb_rva_size != NUM_DIR_ENTRIES )
                ofs << "\tNon conform NumberOfRvaAndSizes value (!= 16)\n";
            ofs << "DataDirectory[16]:\n";
            for ( int i = 0; i < nb_rva_size; ++i ) {
                auto &v = p->peHeader.nt.OptionalHeader.DataDirectory[i];
                ofs << data_directory_values[i].name << ":\t[VA=0x" << std::hex
                    << v.VirtualAddress << " Size=" << std::dec << v.Size
                    << "]\n";
            }
        }
        else {
            DUMP_FIELD( OptionalHeader64.Magic );
            ofs << "\t= ";
            print_flags_value( signature_values,
                               DUMP_VAL( OptionalHeader64.Magic ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader64.MajorLinkerVersion );
            DUMP_FIELD( OptionalHeader64.MinorLinkerVersion );
            DUMP_FIELD( OptionalHeader64.SizeOfCode );
            DUMP_FIELD( OptionalHeader64.SizeOfInitializedData );
            DUMP_FIELD( OptionalHeader64.SizeOfUninitializedData );
            DUMP_FIELD( OptionalHeader64.AddressOfEntryPoint );
            DUMP_FIELD( OptionalHeader64.BaseOfCode );
            DUMP_FIELD( OptionalHeader64.ImageBase );
            DUMP_FIELD( OptionalHeader64.SectionAlignment );
            DUMP_FIELD( OptionalHeader64.FileAlignment );
            DUMP_FIELD( OptionalHeader64.MajorOperatingSystemVersion );
            DUMP_FIELD( OptionalHeader64.MinorOperatingSystemVersion );
            DUMP_FIELD( OptionalHeader64.MajorImageVersion );
            DUMP_FIELD( OptionalHeader64.MinorImageVersion );
            DUMP_FIELD( OptionalHeader64.MajorSubsystemVersion );
            DUMP_FIELD( OptionalHeader64.MinorSubsystemVersion );
            DUMP_FIELD( OptionalHeader64.Win32VersionValue );
            DUMP_FIELD( OptionalHeader64.SizeOfImage );
            DUMP_FIELD( OptionalHeader64.SizeOfHeaders );
            DUMP_FIELD( OptionalHeader64.CheckSum );
            DUMP_FIELD( OptionalHeader64.Subsystem );
            ofs << "\t= ";
            print_flags_value( subsystem_values,
                               DUMP_VAL( OptionalHeader64.Subsystem ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader64.DllCharacteristics );
            ofs << "\t= ";
            print_flags_bit( dll_flags,
                             DUMP_VAL( OptionalHeader64.DllCharacteristics ) );
            ofs << '\n';
            DUMP_FIELD( OptionalHeader64.SizeOfStackReserve );
            DUMP_FIELD( OptionalHeader64.SizeOfStackCommit );
            DUMP_FIELD( OptionalHeader64.SizeOfHeapReserve );
            DUMP_FIELD( OptionalHeader64.SizeOfHeapCommit );
            DUMP_FIELD( OptionalHeader64.LoaderFlags );
            DUMP_FIELD( OptionalHeader64.NumberOfRvaAndSizes );
            auto nb_rva_size = DUMP_VAL( OptionalHeader64.NumberOfRvaAndSizes );
            if ( nb_rva_size != NUM_DIR_ENTRIES )
                ofs << "\tNon conform NumberOfRvaAndSizes value (!= 16)\n";
            ofs << "DataDirectory[16]:\n";
            for ( int i = 0; i < nb_rva_size; ++i ) {
                auto &v = p->peHeader.nt.OptionalHeader64.DataDirectory[i];
                if ( v.Size == 0 && v.VirtualAddress == 0 )
                    continue;
                ofs << data_directory_values[i].name << ":\t[VA=0x" << std::hex
                    << v.VirtualAddress << " Size=" << std::dec << v.Size
                    << "]\n";
            }
        }
        
#undef DUMP_FIELD
#undef DUMP_DEC_FIELD
        ofs << "Sections: "
            << "\n";
        IterSec( p, printSecs, NULL );
        ofs << "Exports: "
            << "\n";
        IterExpVA( p, printExps, NULL );
        ofs << "Imports: "
            << "\n";
        IterImpVAString( p, printImports, NULL );
        ofs << "Symbols (symbol table): "
            << "\n";
        IterSymbols( p, printSymbols, NULL );
        
        
        
        // read the first 8 bytes from the entry point and print them
        VA entryPoint;
        if ( GetEntryPoint( p, entryPoint ) ) {
            ofs << "First 8 bytes from entry point (0x";
            
            ofs << std::hex << entryPoint << "):"
                << "\n";
            for ( std::size_t i = 0; i < 8; i++ ) {
                std::uint8_t b;
                if ( !ReadByteAtVA( p, i + entryPoint, b ) )
                    ofs << " ERR";
                    
                else
                    ofs << " 0x" << std::hex << static_cast<int>( b );
            }
            
            ofs << "\n";
        }
        
        ofs << "Resources: "
            << "\n";
        IterRsrc( p, printRsrc, NULL );
        
        ofs << "Relocations: "
            << "\n";
        IterRelocs( p, printRelocs, NULL );
        
        DestructParsedPE( p );
    }
    else {
        ofs << "Error: " << GetPEErr() << " (" << GetPEErrString() << ")"
            << "\n";
        ofs << "Location: " << GetPEErrLoc() << "\n";
    }
    ofs.close();
    
    return 0;
}
