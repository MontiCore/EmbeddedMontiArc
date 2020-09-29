#include "utility.h"
#include <iomanip>
#include <locale>
#include <codecvt>
#include <unicorn/unicorn.h>
#if defined _WIN32 || defined _WIN64
    #include <WinSock2.h>
    #include <WS2tcpip.h>
    #pragma comment(lib, "Ws2_32.lib")
    #include <iostream>
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

void Library::init(const FS::File& file) {
    auto name = file.as_system_path() + ".dll";
    handle = LoadLibraryA( name.c_str() );
    Log::info << "Init Library: " << name << "\n";
    if (!loaded())
        throw_lasterr("Cannot open dll " + name);
}

void *Library::get_function( const char *name ) {
    if (!loaded())
        throw_error("Library::get_function() on unloaded library.");
    auto res = GetProcAddress((HMODULE)handle, name);
    if (res == nullptr)
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


void Library::init( const FS::File &file  ) {
    auto name = file.as_system_path() + ".so";
    handle = dlopen( name.c_str(), RTLD_NOW );
    if (!loaded())
        throw_system_error("dlopen() error opening "+name+": "+ dlerror());
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


namespace FS {

    void fill_directories_files(const Directory& directory, std::vector<Directory>* directories, std::vector<File>* files);

    Directory::Directory(const std::string& p) {
        this->path = "";
        if (p.size() == 0) return;
        auto parts = STR::split(p, '\\', '/');
        bool first = true;
        for (auto& e : parts) {
            if (e.size() > 0) {
                this->path += first ? e : "/" + e;
                first = false;
            }
            else if (first) this->path += '/'; //Preserve absolute path
        }
        if (this->path.size() >= 3 && this->path[1] == ':' && this->path[2] != '/')
            throw SystemException("Using drive letter with relative path: " + this->path);
    }




    Directory operator+(const Directory& folder, const Directory& folder2)
    {
        if (folder2.is_absolute() && folder.get_path().size() > 0)
            throw SystemException("Called operator+(Directory(" + folder.get_path() + "), Directory(" + folder2.get_path() + ")): " + folder2.get_path() + " must be a relative directory.");
        if (folder.path.size() == 0) return folder2;
        if (folder2.path.size() == 0) return folder;
        Directory d;
        d.path = folder.path + "/" + folder2.path;
        return d;
    }

    bool Directory::is_absolute() const
    {
        return
            (path.size() >= 2 && path[1] == ':' && STR::is_basic_letter(path[0])) ||
            (path.size() > 0 && (path[0] == '/' || path[0] == '~'));
    }

    DirectoryContent Directory::get_contents() const
    {
        DirectoryContent c;
        fill_directories_files(*this, &c.directories, &c.files);
        return c;
    }

    std::vector<File> Directory::get_files() const
    {
        std::vector<File> f;
        fill_directories_files(*this, nullptr, &f);
        return f;
    }

    std::vector<Directory> Directory::get_directories() const
    {
        std::vector<Directory> d;
        fill_directories_files(*this, &d, nullptr);
        return d;
    }

    std::string Directory::get_name() const
    {
        auto i = path.size();
        while (i > 0 && path[i - 1] != '/') --i;
        return path.substr(i, std::string::npos);
    }

    std::string Directory::to_string() const
    {
        return path;
    }

    File::File(const std::string& folder, const std::string& name) : File(Directory(folder), name) {}

    File::File(const std::string& name) : File(Directory(), name) {}



    File::File(const Directory& folder, const std::string& name) : folder(folder) {
        sint i = (sint)name.size() - 1;
        while (i >= 0 && name[i] != '/' && name[i] != '\\') --i;
        if (i >= 0) {
            this->folder = folder + Directory(name.substr(0, i));
            this->name = name.substr(i + 1LL, std::string::npos);
        }
        else {
            this->folder = folder;
            this->name = name;
        }
        if (this->name.size() == 0) throw SystemException("Creating File with empty name.");
        i = (sint)this->name.size() - 1;
        while (i >= 0 && this->name[i] != '.') --i;
        this->ext_pos = i >= 0 ? (uint)i : (uint)this->name.size();
    }

    File operator+(const Directory& folder, const File& file)
    {
        File f;
        f.folder = folder + file.folder;
        f.name = file.name;
        f.ext_pos = file.ext_pos;
        return f;
    }

    std::string File::to_string() const
    {
        if (folder.get_path().size() > 0)
            return folder.to_string() + "/" + name;
        return name;
    }

    

#if defined _WIN32 || defined _WIN64


    std::string File::as_system_path() const
    {
        if (folder.get_path().size() > 0)
            return folder.as_system_path() + name;
        else return name;
    }


    Directory current_directory()
    {
        TCHAR buff[MAX_PATH];
        if (GetCurrentDirectory(sizeof(buff), buff) != 0)
            return Directory(buff);
        throw_lasterr("GetCurrentDirectory");
    }

    Directory Directory::canonical() const
    {
        /*if (!is_absolute())
            throw_system_error("Directory.canonical() only supported on absolute paths. (Called on "+to_string()+")");*/
        TCHAR buff[MAX_PATH];
        if (PathCanonicalizeA(buff, as_system_path().c_str())) {
            return Directory(buff);
        }
        throw_lasterr("PathCanonicalizeA");
    }

    bool Directory::exists() const
    {
        throw SystemException("Not implemented");
    }

    bool Directory::mkdir() const
    {
        throw SystemException("Not implemented");
    }



    std::string Directory::as_system_path() const
    {
        std::string p = path;
        for (auto& c : p) {
            if (c == '/') c = '\\';
        }
        return p.size() > 0 ? p + '\\' : "";
    }

    bool File::exists() const
    {
        throw SystemException("Not implemented");
    }

    




    void fill_directories_files(const Directory& directory, std::vector<Directory>* directories, std::vector<File>* files) {
        WIN32_FIND_DATA fdFile;
        HANDLE hFind = NULL;

        //std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
        //std::wstring wide = converter.from_bytes(folder);
        if ((hFind = FindFirstFile((directory.as_system_path() + "*").c_str(), &fdFile)) == INVALID_HANDLE_VALUE)
            throw_lasterr("fill_directories_files()");

        do {
            std::string file_name = (const char*)fdFile.cFileName;
            if (file_name != "." && file_name != "..") {
                if (fdFile.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
                    if (directories) directories->emplace_back(Directory(file_name));
                }
                else {
                    if (files) files->emplace_back(File(file_name));
                }
            }
        } while (FindNextFile(hFind, &fdFile));

        FindClose(hFind);
    }

    

#else

    std::string File::as_system_path() const
    {
        if (folder.get_path().size() > 0)
            return folder.as_system_path() + '/' + name;
        else return name;
    }

    Directory current_directory()
    {
        auto cwd = get_current_dir_name();
        if (cwd != NULL){
            Directory d(cwd);
            free(cwd);
            return d;
        }
        throw_lasterr("get_current_dir_name()");
    }

    Directory Directory::canonical() const
    {
        //if (!is_absolute())
        //    throw_system_error("Directory.canonical() only supported on absolute paths.");

        char actualpath[PATH_MAX];
        if (realpath(path.c_str(), actualpath) != NULL)
            return Directory(actualpath);
        throw_lasterr("realpath()");
    }

    bool Directory::exists() const
    {
        throw SystemException("Not implemented");
    }

    bool Directory::mkdir() const
    {
        throw SystemException("Not implemented");
    }



    std::string Directory::as_system_path() const
    {
        return path;
    }

    bool File::exists() const
    {
        throw SystemException("Not implemented");
    }

    




    void fill_directories_files(const Directory& directory, std::vector<Directory>* directories, std::vector<File>* files) {
        DIR* d;
        struct dirent* dir;
        d = opendir(directory.as_system_path().c_str());
        if (!d)
            throw_lasterr("opendir()");
        while ((dir = readdir(d)) != NULL) {
            if (dir->d_type == DT_REG) {
                if (files) files->emplace_back(File(directory, dir->d_name));
            } else if (dir->d_type == DT_DIR){
                if (directories) directories->emplace_back(directory + Directory(dir->d_name));
            }
        }
        closedir(d);
    }

#endif







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



ulong BIT_MASKS[65] = {
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
