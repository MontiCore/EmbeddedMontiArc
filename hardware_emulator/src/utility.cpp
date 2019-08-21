/**
 *
 *  ******************************************************************************
 *  MontiCAR Modeling Family, www.se-rwth.de
 *  Copyright (c) 2017, Software Engineering Group at RWTH Aachen,
 *  All rights reserved.
 *
 *  This project is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 3.0 of the License, or (at your option) any later version.
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this project. If not, see <http://www.gnu.org/licenses/>.
 * *******************************************************************************
 */
#include "utility.h"
#include <iomanip>

#if defined _WIN32 || defined _WIN64
    #include <WinSock2.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include <iostream>
    #include "windows.h"
    #include "shlwapi.h"
    #include <DbgHelp.h>
#else
    #include <dlfcn.h>
    #include <limits.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <dirent.h>
#endif

uint BIT_MASKS[] = {
    0b0,
    0b1,                0b11,               0b111,              0b1111,
    0b11111,            0b111111,           0b1111111,          0b11111111,
    0b111111111,        0b1111111111,       0b11111111111,      0b111111111111,
    0b1111111111111,    0b11111111111111,   0b111111111111111,  0b1111111111111111
};


void Utility::write_uint64_t( char *mem_pos, uint64_t value ) {
    for ( auto i : ulrange( sizeof( uint64_t ) ) ) {
        uint64_t sub = ( value >> ( i * 8 ) ) & BIT_MASKS[8];
        mem_pos[i] = *( ( char * )&sub );
    }
}

uint64_t Utility::read_uint64_t( char *mem_pos ) {
    uint64_t res = 0;
    for ( auto i : ulrange( sizeof( uint64_t ) ) ) {
        uint64_t sub = 0;
        *( ( char * )&sub ) = mem_pos[i];
        res |= sub << ( i * 8 );
    }
    return res;
}

void Utility::write_uint32_t( char *mem_pos, uint32_t value ) {
    for ( auto i : ulrange( sizeof( uint32_t ) ) ) {
        uint32_t sub = ( value >> ( i * 8 ) ) & BIT_MASKS[8];
        mem_pos[i] = *( ( char * )&sub );
    }
}

uint32_t Utility::read_uint32_t( char *mem_pos ) {
    uint32_t res = 0;
    for ( auto i : ulrange( sizeof( uint32_t ) ) ) {
        uint32_t sub = 0;
        *( ( char * )&sub ) = mem_pos[i];
        res |= sub << ( i * 8 );
    }
    return res;
}


void Array<bool>::resize( uint new_size ) {
    if ( new_size == m_size )
        return;
        
    if ( new_size == 0 ) {
        drop();
        return;
    }
    
    if ( m_size == 0 ) {
        init( new_size );
        return;
    }
    
    uint new_array_size = get_array_size( new_size );
    data.resize( new_array_size );
    m_size = new_size;
}


#if defined _WIN32 || defined _WIN64
void *ConsoleColor::Console::hstdout = 0;
uchar ConsoleColor::Console::csbi[22];


void ConsoleColor::Console::init() {
    hstdout = GetStdHandle( STD_OUTPUT_HANDLE );
    
    // Remember how things were when we started
    GetConsoleScreenBufferInfo( hstdout, ( PCONSOLE_SCREEN_BUFFER_INFO )&csbi );
}

void ConsoleColor::Console::drop() {
    SetConsoleTextAttribute( hstdout, ( ( PCONSOLE_SCREEN_BUFFER_INFO )&csbi )->wAttributes );
}

void ConsoleColor::Console::set_color( Color which ) {
    SetConsoleTextAttribute( hstdout, which.get() );
}

void ConsoleColor::Console::test_color() {
    for ( uint i : urange( 16 ) ) {
        std::cout << '\t';
        for ( uint j : urange( 16 ) ) {
            uint v = i + j * 16;
            ConsoleColor::Console::set_color( Color( ( ColorValue )i, ( ColorValue )j ) );
            std::cout << " ";
            std::cout << std::setw( 3 ) << v;
            std::cout << " ";
            ConsoleColor::Console::set_color( ConsoleColor::DEFAULT );
            std::cout << " ";
        }
        std::cout << std::endl;
    }
}
#else

const char *ConsoleColor::BLACK = "\033[30m";
const char *ConsoleColor::DARK_BLUE = "\033[34m";
const char *ConsoleColor::DARK_GREEN = "\033[32m";
const char *ConsoleColor::TURQUOISE = "\033[36m";
const char *ConsoleColor::DARK_RED = "\033[31m";
const char *ConsoleColor::PURPLE = "\033[35m";
const char *ConsoleColor::DARK_YELLOW = "\033[33m";
const char *ConsoleColor::LIGHT_GRAY = "\033[37m";
const char *ConsoleColor::DARK_GRAY = "\033[90m";
const char *ConsoleColor::BLUE = "\033[94m";
const char *ConsoleColor::GREEN = "\033[92m";
const char *ConsoleColor::LIGHT_BLUE = "\033[96m";
const char *ConsoleColor::RED = "\033[91m";
const char *ConsoleColor::PINK = "\033[95m";
const char *ConsoleColor::YELLOW = "\033[93m";
const char *ConsoleColor::WHITE = "\033[97m";
const char *ConsoleColor::DEFAULT = "\033[0m";

void ConsoleColor::Console::drop() {
    std::cout << DEFAULT;
}

void ConsoleColor::Console::set_color( Color which ) {
    std::cout << which.get();
}



#endif
#if defined _WIN32 || defined _WIN64

bool undercorate_function_name( const std::string &name, Array<char> &buffer ) {
    return UnDecorateSymbolName( name.c_str(), buffer.begin(), buffer.size(), UNDNAME_COMPLETE );
}
#else
bool undercorate_function_name( const std::string &name, Array<char> &buffer ) {
    return false;
}

#endif

ConsoleColor::Color Log::current_color = ConsoleColor::DEFAULT;
Log::TagStruct Log::tag;
Log::LogStream Log::info( ConsoleColor::DEFAULT, "[I]" );
Log::LogStream Log::err( ConsoleColor::RED, "[ERR]" );
Log::LogStream Log::debug( ConsoleColor::LIGHT_BLUE, "[DBG]" );
Log::LogStream Log::sys( ConsoleColor::YELLOW, "[SYS]" );
Log::LogStream Log::mem_read( ConsoleColor::BLUE, "[R]" );
Log::LogStream Log::mem_write( ConsoleColor::LIGHT_BLUE, "[W]" );
Log::LogStream Log::mem_fetch( ConsoleColor::PINK, "[F]" );
Log::LogStream Log::code( ConsoleColor::GREEN, "[C]" );
Log::LogStream Log::reg( ConsoleColor::DARK_GRAY, "[REG]" );
Log::LogStream Log::new_val( ConsoleColor::DARK_YELLOW, "[NEW]" );
Log::LogStream Log::note( ConsoleColor::PINK, "[N]" );
Log::LogStream Log::white( ConsoleColor::WHITE, "" );
Log::LogStream Log::test( ConsoleColor::GREEN, "" );


#if defined _WIN32 || defined _WIN64

bool Library::init( const char *name ) {
    handle = LoadLibraryA( name );
    return loaded();
}

void *Library::get_function( const char *name ) {
    if ( loaded() )
        return GetProcAddress( ( HMODULE )handle, name );
    return nullptr;
}

Library::~Library() {
    if ( loaded() ) {
        FreeLibrary( ( HMODULE ) handle );
        handle = nullptr;
    }
}

#else


bool Library::init( const char *name ) {
    auto n = name + std::string(".so");
    handle = dlopen( n.c_str(), RTLD_NOW );
    if (!loaded()){
        Log::err << Log::tag << "dlopen() error: " << dlerror() << "\n";
        return false;
    }
    return true;
}

void *Library::get_function( const char *name ) {
    if ( loaded() )
        return dlsym( handle, name );
    return nullptr;
}

Library::~Library() {
    if ( loaded() ) {
        dlclose( handle );
        handle = nullptr;
    }
}

#endif

FS::File::File(std::string folder, std::string name){
    this->path = append(folder, name);
    int dot_pos = -1;
    for (int i = name.size()-1; i >= 0; --i){
        if (name[i] == '.') {
            dot_pos = i;
            break;
        }
    }
    if (dot_pos != -1){
        this->name = name.substr(0, dot_pos);
        this->extension = name.substr(dot_pos);
    } else {
        this->name = name;
    }
}

#if defined _WIN32 || defined _WIN64
std::string FS::append(std::string path, std::string file){
    if (path.size() == 0) return file;
    return path[path.size()-1] == '\\' ? path + file : path + '\\' + file;
}

std::string FS::current_path(){
    TCHAR buff[MAX_PATH];
    if (GetCurrentDirectory(sizeof(buff), buff) != 0)
        return std::string(buff);
    std::cerr << "GetCurrentDirectory() error" << std::endl;
    return "";
}

std::string FS::canonical(const std::string &path){
    TCHAR buff[MAX_PATH];
    if (PathCanonicalizeA(buff, path.c_str()))
        return std::string(buff);
    std::cerr << "PathCanonicalizeA() error" << std::endl;
    return "";
}
std::list<File> FS::directory_files( std::string folder ){
    
}
#else
std::string FS::append(std::string path, std::string file){
    if (path.size() == 0) return '/' + file;
    return path[path.size()-1] == '/' ? path + file : path + '/' + file;
}

std::string FS::current_path(){
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != NULL)
        return std::string(cwd);
    perror("getcwd() error");
    return "";
}

std::string FS::canonical(const std::string &path){
    char actualpath [PATH_MAX];
    if (realpath(path.c_str(), actualpath) != NULL)
        return std::string(actualpath);
    perror("realpath() error");
    return "";
}
std::list<FS::File> FS::directory_files( std::string folder ){
    std::list<File> files;
    DIR *d;
    struct dirent *dir;
    d = opendir(folder.c_str());
    if (d) {
        while ((dir = readdir(d)) != NULL) {
            if (dir->d_type == DT_REG){
                files.emplace_back(File(folder, dir->d_name));
            }
        }
        closedir(d);
    } else {
        perror("opendir() error");
    }
    return files;
}

#endif

