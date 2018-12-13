#include "utility.h"
#include <iomanip>

#if defined _WIN32 || defined _WIN64
    #include <WinSock2.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include <iostream>
    #include "windows.h"
#endif

uint BIT_MASKS[] = {
    0b0,
    0b1,                0b11,               0b111,              0b1111,
    0b11111,            0b111111,           0b1111111,          0b11111111,
    0b111111111,        0b1111111111,       0b11111111111,      0b111111111111,
    0b1111111111111,    0b11111111111111,   0b111111111111111,  0b1111111111111111
};


void Utility::write_uint64_t( char *mem_pos, uint64_t value ) {
    for ( auto i : Range( sizeof( uint64_t ) ) ) {
        uint64_t sub = ( value >> ( i * 8 ) ) & BIT_MASKS[8];
        mem_pos[i] = *( ( char * )&sub );
    }
}

uint64_t Utility::read_uint64_t( char *mem_pos ) {
    uint64_t res = 0;
    for ( auto i : Range( sizeof( uint64_t ) ) ) {
        uint64_t sub = 0;
        *( ( char * )&sub ) = mem_pos[i];
        res |= sub << ( i * 8 );
    }
    return res;
}

void Utility::color_def() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::DEFAULT ) );
}

void Utility::color_err() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::RED ) );
}

void Utility::color_sys() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::YELLOW ) );
}

void Utility::color_mem_write() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::LIGHT_BLUE ) );
}

void Utility::color_mem_fetch() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::PINK ) );
}

void Utility::color_mem_read() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::BLUE ) );
}

void Utility::color_code() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::GREEN ) );
}

void Utility::color_reg() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::DARK_GRAY ) );
}

void Utility::color_new() {
    ConsoleColor::Console::set_color( ConsoleColor::Color( ConsoleColor::DARK_YELLOW ) );
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


#ifdef WIN32
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
    for ( uint i : Range( 16 ) ) {
        std::cout << '\t';
        for ( uint j : Range( 16 ) ) {
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
#endif

#include <DbgHelp.h>

bool undercorate_function_name( const std::string &name, Array<char> &buffer ) {
    return UnDecorateSymbolName( name.c_str(), buffer.begin(), buffer.size(), UNDNAME_COMPLETE );
}
