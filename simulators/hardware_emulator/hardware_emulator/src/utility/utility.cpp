/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "utility.h"
#include <iomanip>
#include <locale>
#include <codecvt>
#include <iostream>
#include <unicorn/unicorn.h>
#if defined _WIN32 || defined _WIN64
    #include <WinSock2.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include <windows.h>
    #include <shlwapi.h>
    #include <DbgHelp.h>
#else
    #include <dirent.h>
    #include <dlfcn.h>
    #include <limits.h>
    #include <stdlib.h>
    #include <unistd.h>
#endif


AssertionFailureException::AssertionFailureException(const char* expression, const char* file, int line,
    const char* message) {
    std::ostringstream outputStream;

    std::string message_string(message);
    if (!message_string.empty())
        outputStream << message_string << ": ";

    std::string expressionString = expression;
    if (expressionString == "false" || expressionString == "0" || expressionString == "FALSE")
        outputStream << "Unreachable code assertion";
    else
        outputStream << "Assertion '" << expression << "'";

    outputStream << " failed in file '" << file << "' line " << line;
    Log::err.log_tag("%s", outputStream.str().c_str());
}

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


void vector_bool::resize( uint new_size ) {
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


Console uconsole;


#if defined _WIN32 || defined _WIN64

Console::Console() {
    hstdout = GetStdHandle( STD_OUTPUT_HANDLE );
    GetConsoleScreenBufferInfo( hstdout, ( PCONSOLE_SCREEN_BUFFER_INFO )&csbi );
    
    current_color.text_color = ( ConsoleColor::ColorValue ) - 1;
    current_color.bg_color = ( ConsoleColor::ColorValue ) - 1;
    set_color( ConsoleColor::DEFAULT );
}

Console::~Console() {
    SetConsoleTextAttribute( hstdout, ( ( PCONSOLE_SCREEN_BUFFER_INFO )&csbi )->wAttributes );
}

void Console::set_color( ConsoleColor which ) {
    if ( current_color != which ) {
        current_color = which;
        SetConsoleTextAttribute( hstdout, which.get() );
    }
}

void Console::test_color() {
    for ( auto i : irange( 16 ) ) {
        std::cout << '\t';
        for ( auto j : irange( 16 ) ) {
            auto v = i + j * 16;
            set_color( ConsoleColor( ( ConsoleColor::ColorValue )i, ( ConsoleColor::ColorValue )j ) );
            std::cout << " ";
            std::cout << std::setw( 3 ) << v;
            std::cout << " ";
            set_color( ConsoleColor::DEFAULT );
            std::cout << " ";
        }
        std::cout << std::endl;
    }
}



#else

Console::Console() {
    current_color.text_color = ( ConsoleColor::ColorValue ) 0;
    current_color.bg_color = ( ConsoleColor::ColorValue ) 0;
}

Console::~Console() {
    set_color( ConsoleColor::DEFAULT );
}


void Console::set_color( ConsoleColor which ) {
    if ( which.text_color != current_color.text_color ) {
        if ( which.bg_color != current_color.bg_color )
            std::cout << "\033[" << which.text_color << ";" << ( which.bg_color + 10 ) << "m";
        else
            std::cout << "\033[" << which.text_color << "m";
    }
    else if ( which.bg_color != current_color.bg_color )
        std::cout << "\033[" << ( which.bg_color + 10 ) << "m";
    current_color = which;
}

void Console::test_color() {
    for ( auto u : irange( 2 ) ) {
        sint ubase = u == 0 ? 30 : 90;
        for ( auto i : irange( 8 ) ) {
            std::cout << '\t';
            for ( auto v : irange( 2 ) ) {
                sint vbase = v == 0 ? 30 : 90;
                for ( auto j : irange( 8 ) ) {
                    set_color( ConsoleColor( ( ConsoleColor::ColorValue )( ubase + i ), ( ConsoleColor::ColorValue )( vbase + j ) ) );
                    std::cout << " ";
                    std::cout << std::setw( 3 ) << v;
                    std::cout << " ";
                    set_color( ConsoleColor::DEFAULT );
                    std::cout << " ";
                }
            }
            std::cout << std::endl;
        }
    }
    
}


#endif

#if defined _WIN32 || defined _WIN64

bool undercorate_function_name( const std::string &name, std::vector<char> &buffer ) {
    return UnDecorateSymbolName( name.c_str(), buffer.data(), (DWORD) buffer.size(), UNDNAME_COMPLETE );
}
#else
bool undercorate_function_name( const std::string &name, std::vector<char> &buffer ) {
    return false;
}

#endif

Log::TagStruct Log::tag;
Log::LogStream Log::info( ConsoleColor::DEFAULT, "[I]", "info" );
Log::LogStream Log::err( ConsoleColor::RED, "[ERR]", "err" );
Log::LogStream Log::debug( ConsoleColor::LIGHT_BLUE, "[DBG]", "debug" );
Log::LogStream Log::sys( ConsoleColor::YELLOW, "[SYS]", "system" );
Log::LogStream Log::mem_read( ConsoleColor::BLUE, "[R]", "debug_mem_read" );
Log::LogStream Log::mem_write( ConsoleColor::LIGHT_BLUE, "[W]", "debug_mem_write" );
Log::LogStream Log::mem_fetch( ConsoleColor::PINK, "[F]", "debug_mem_fetch" );
Log::LogStream Log::code( ConsoleColor::GREEN, "[C]", "debug_code" );
Log::LogStream Log::instruction_operands( ConsoleColor::GREEN,"[O]", "debug_instruction");
Log::LogStream Log::cache_hit_ratio( ConsoleColor::DARK_YELLOW, "[CACHE]", "debug_cache_hit_ratio");
Log::LogStream Log::mem_access( ConsoleColor::DARK_YELLOW, "[M]", "debug_mem_access");
Log::LogStream Log::reg( ConsoleColor::DARK_GRAY, "[REG]", "debug_register" );
Log::LogStream Log::new_val( ConsoleColor::DARK_YELLOW, "[NEW]", "debug_new" );
Log::LogStream Log::note( ConsoleColor::PINK, "[N]", "note" );
Log::LogStream Log::white( ConsoleColor::WHITE, "", "info" );
Log::LogStream Log::test( ConsoleColor::GREEN, "", "test" );
Log::LogStream Log::ap(ConsoleColor::DARK_YELLOW, "[AP]", "autopilot");

std::unique_ptr<Log::OStreamTarget> Log::output_stream = std::make_unique<Log::STDOutput>();

char Log::LARGE_BUFFER[Log::LARGE_BUFFER_SIZE];

#if defined _WIN32 || defined _WIN64

void Library::init(const fs::path& file) {
    auto new_file = file;
    new_file += fs::path(".dll");
    auto name = new_file.string();
    handle = LoadLibraryA( name.c_str() );
    Log::info.log_tag("Init Library: %s", name.c_str());
    if (!loaded())
        throw_lasterr("Cannot open dll " + name);
}

void *Library::get_function( const char *name, bool optional ) {
    if (!loaded())
        throw_error("Library::get_function() on unloaded library.");
    auto res = GetProcAddress((HMODULE)handle, name);
    if (res == nullptr && !optional)
        throw_lasterr("GetProcAddress(" + std::string(name) + ")");
    return res;
}

Library::~Library() {
    if ( loaded() ) {
        FreeLibrary( ( HMODULE ) handle );
        handle = nullptr;
    }
}

std::string get_last_error_str() {
    TCHAR buff[1024];
    DWORD dw = GetLastError();

    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL,
        dw, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        buff, 1024, NULL);
    return std::string(buff);
}

#else


void Library::init( const fs::path &file  ) {
    auto new_file = file;
    new_file += fs::path(".so");
    auto name = new_file.string();
    handle = dlopen( name.c_str(), RTLD_NOW );
    if (!loaded())
        throw_system_error("dlopen() error opening "+name+": "+ dlerror());
}

void *Library::get_function( const char *name, bool optional ) {
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

std::string get_last_error_str() {
    auto msg = strerror(errno);
    return std::string(msg);
}

#endif


namespace STR {

    std::vector<std::string> split(const std::string& input, const char& delim, const char& delim2) {
        std::vector<std::string> res;
        size_t pos = 0;
        for (auto i : range<size_t>(input.size())) {
            if (input[i] == delim || input[i] == delim2) {
                res.emplace_back(input.substr(pos, i - pos));
                pos = i + 1;
            }
        }
        res.emplace_back(input.substr(pos, input.size() - pos));
        return res;
    }

    std::vector<std::string> split(const std::string& input, const char& delim) {
        std::vector<std::string> res;
        size_t pos = 0;
        for (auto i : range<size_t>(input.size())) {
            if (input[i] == delim) {
                res.emplace_back(input.substr(pos, i - pos));
                pos = i + 1;
            }
        }
        res.emplace_back(input.substr(pos, input.size() - pos));
        return res;
    }

    bool is_basic_letter(const char& c) {
        //65 (A) -> 90 (Z), 97 (a) -> 122 (z)
        return (c >= 65 && c <= 90) || (c >= 97 && c <= 122);
    }
}




SystemException SystemException::lasterr(const std::string& description, const char* file, int line)
{
    return SystemException(description + ("\n\tSystem error: " + get_last_error_str()), file, line);
}

SystemException SystemException::lasterr()
{
    return SystemException("SystemException: " + get_last_error_str());
}

std::string Error::jni_error(const std::string& description)
{
    return "JNI Error: \n\t" + description;
}

std::string Error::hardware_emu_init_error(const std::string& description)
{
    return "HardwareEmulator initialization error: \n\t" + description;
}

std::string Error::hardware_emu_software_load_error(const std::string& description)
{
    return "HardwareEmulator: Error loading software: \n\t" + description;
}

std::string Error::direct_sim_software_load_error(const std::string& description)
{
    return "DirectSimulator: Error loading software: \n\t" + description;
}


bool json_get(const json& j, const char* entry, bool& target) {
    if (!j.contains(entry)) return false;
    auto& e = j[entry];
    if (!e.is_boolean()) return false;
    target = e.get<int>();
    return true;
}
bool json_get(const json& j, const char* entry, int& target) {
    if (!j.contains(entry)) return false;
    auto& e = j[entry];
    if (!e.is_number_integer()) return false;
    target = e.get<int>();
    return true;
}
bool json_get(const json& j, const char* entry, uint& target) {
    if (!j.contains(entry)) return false;
    auto& e = j[entry];
    if (!e.is_number_unsigned()) return false;
    target = e.get<uint>();
    return true;
}

bool json_get(const json& j, const char* entry, ulong& target) {
    if (!j.contains(entry)) return false;
    auto& e = j[entry];
    if (!e.is_number_integer()) return false;
    target = e.get<ulong>();
    return true;
}

bool json_get(const json& j, const char* entry, std::string& target) {
    if (!j.contains(entry)) return false;
    auto& e = j[entry];
    if (!e.is_string()) return false;
    target = e.get<std::string>();
    return true;
}



unsigned int long long BIT_MASKS[65] = {
    0x0LL,
    0x1LL, 0x3LL, 0x7LL, 0xfLL,
    0x1fLL, 0x3fLL, 0x7fLL, 0xffLL,
    0x1ffLL, 0x3ffLL, 0x7ffLL, 0xfffLL,
    0x1fffLL, 0x3fffLL, 0x7fffLL, 0xffffLL,
    0x1ffffLL, 0x3ffffLL, 0x7ffffLL, 0xfffffLL,
    0x1fffffLL, 0x3fffffLL, 0x7fffffLL, 0xffffffLL,
    0x1ffffffLL, 0x3ffffffLL, 0x7ffffffLL, 0xfffffffLL,
    0x1fffffffLL, 0x3fffffffLL, 0x7fffffffLL, 0xffffffffLL,
    0x1ffffffffLL, 0x3ffffffffLL, 0x7ffffffffLL, 0xfffffffffLL,
    0x1fffffffffLL, 0x3fffffffffLL, 0x7fffffffffLL, 0xffffffffffLL,
    0x1ffffffffffLL, 0x3ffffffffffLL, 0x7ffffffffffLL, 0xfffffffffffLL,
    0x1fffffffffffLL, 0x3fffffffffffLL, 0x7fffffffffffLL, 0xffffffffffffLL,
    0x1ffffffffffffLL, 0x3ffffffffffffLL, 0x7ffffffffffffLL, 0xfffffffffffffLL,
    0x1fffffffffffffLL, 0x3fffffffffffffLL, 0x7fffffffffffffLL, 0xffffffffffffffLL,
    0x1ffffffffffffffLL, 0x3ffffffffffffffLL, 0x7ffffffffffffffLL, 0xfffffffffffffffLL,
    0x1fffffffffffffffLL, 0x3fffffffffffffffLL, 0x7fffffffffffffffLL, 0xffffffffffffffffLL
};
