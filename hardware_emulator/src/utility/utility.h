/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once
#include <inttypes.h>
#include <exception>
#include <string>
#include <sstream>
#include <fstream>
#include <memory>
#include <cstring>
#include <cmath>
#include <list>
#include <vector>
#include <chrono>
#include <stdarg.h>
#include <filesystem>

#include "json.hpp"
using json = nlohmann::json;
namespace fs = std::filesystem;

namespace Utility {

    /*
        These read and write functions are Little-Endian
    */

    void write_uint64_t( char *mem_pos, uint64_t value );
    uint64_t read_uint64_t( char *mem_pos );
    
    void write_uint32_t( char *mem_pos, uint32_t value );
    uint32_t read_uint32_t( char *mem_pos );
}




#include <cstdint>
/*
    Short mnemonics for signed and unsigned integers
*/
using schar = int8_t;
using sbyte = int8_t;
using sshort = int16_t;
using sint = int32_t;
using slong = int64_t;

using uchar = uint8_t;
using ubyte = uint8_t;
using ushort = uint16_t;
using uint = uint32_t;
using ulong = uint64_t;

/*
    Bit manipulation macros
    BIT(pos)            gives a number with the pos'th bit at 1
    isBit[High/Low]     check if the pos-th bit in var is high/low
    setBit[High/Low]    sets the pos-th bit in var high/low
    isFlagHigh          checks if a bit true in flag is also true in var
    setFlag[HighLow]    sets the bits true in flag as high/low in var
*/
#define             BIT(pos)                ((ulong)1 << (pos))

#define             isBitHigh(var, pos)     ((var >> (pos)) & 1)
#define             isBitLow(var, pos)      (!((var >> (pos)) & 1))
#define             setBitHigh(var, pos)    var |= (((ulong)1) << (pos))
#define             setBitLow(var, pos)     var &= ~(((ulong)1) << (pos))

#define             isFlagHigh(var, flag)   (var & static_cast<ulong>(flag))
#define             setFlagHigh(var, flag)  var |= static_cast<ulong>(flag)
#define             setFlagLow(var, flag)   var &= ~(static_cast<ulong>(flag))

// array of 0b1, 0b11, 0b111, ... , 0b111...111
extern unsigned int long long BIT_MASKS[65];



/*
    DEBUG MODE ASSERTIONS
*/

struct AssertionFailureException : public std::exception {
    AssertionFailureException(const char* expression, const char* file, int line,
        const char* message);
};

// Assert that EXPRESSION evaluates to true, otherwise raise AssertionFailureException with associated MESSAGE
#ifdef _DEBUG
    #define throw_assert(EXPRESSION, MESSAGE) if(!(EXPRESSION)) { throw AssertionFailureException(#EXPRESSION, __FILE__, __LINE__,  MESSAGE); }
#else
    #define throw_assert(E, M) ((void)0)
#endif

/*
    Exception Type for System specific errors.
    lasterr() will read the details of the last system error (GetLastError() under Windows) and return an exception that
    can be thrown.
*/
class SystemException : public std::exception {
    std::string description;
public:
    SystemException(const std::string& description) : description("SystemError: " + description) {}
    SystemException(const std::string& description, const char* file, int line) : description(
        "C++ Simulator Error at " + std::string(file) + "[line: " + std::to_string(line) + "]: " + description
    ) {}
    static SystemException lasterr(const std::string& description, const char* file, int line);
    static SystemException lasterr();
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};

/*
    Exception type with more flexible std::string based description.
*/
class SimulatorException : public std::exception {
    std::string description;
public:
    SimulatorException(const std::string& description) : description("C++ Simulator Error: " + description) {}
    SimulatorException(const std::string& description, const char* file, int line) : description(
        "C++ Simulator Error at " + std::string(file) + "[line: " + std::to_string(line) + "]: " + description
    ) {}
    virtual const char* what() const throw()
    {
        return description.c_str();
    }
};

/*
    These macros record the file and line at which they are called into the exception description.
*/
#define throw_system_error(MESSAGE) throw SystemException(MESSAGE, __FILE__, __LINE__)
#define throw_lasterr(MESSAGE) throw SystemException::lasterr(MESSAGE, __FILE__, __LINE__)
#define throw_error(MESSAGE) throw SimulatorException(MESSAGE, __FILE__, __LINE__)

namespace Error {
    /*
        Formats Exception descriptions for different contexts.
    */
    std::string jni_error(const std::string& description);
    std::string hardware_emu_init_error(const std::string &description);
    std::string hardware_emu_software_load_error(const std::string& description);
    std::string direct_sim_software_load_error(const std::string& description);
}


template<typename T>
struct RangeIterator {
    T val;
    RangeIterator( T val ) : val( val ) {}
    RangeIterator() : val( 0 ) {}
    RangeIterator operator++() {
        return ++val;
    }
    T operator*() {
        return val;
    }
    bool operator !=( const RangeIterator &other ) {
        return other.val != val;
    }
};
/*
    RANGE

    Describes a range of numbers that implements the begin() and end() functions, which allows the range
    to be used as follows in 'for' loops:

    for (auto i : range<T>(1,4)){
        ...
    }

*/
template<typename T>
class range {
    public:
    
        range( T start, T end ) : m_start( start ), m_end( end ) {}
        range() : m_start( 0 ), m_end( 0 ) {}
        range( T size ) : m_start( 0 ), m_end( size ) {}
        RangeIterator<T> begin() {
            return m_start;
        }
        RangeIterator<T> end() {
            return m_end;
        }
    private:
    
        T m_start;
        T m_end;
};

/*
    Mnemonics for various range iterator types
*/
using urange = range<uint>;
using irange = range<sint>;
using srange = range<sshort>;
using usrange = range<ushort>;
using lrange = range<slong>;
using ulrange = range<ulong>;
using crange = range<schar>;
using ucrange = range<uchar>;







/*
    An ArraySlice proxies the content or a part of the content of an other array or array slice.
    It overloads the operator[] and the iteration functions.
*/
template<typename T>
class vector_slice {
        T *ref;
        uint m_start;
        uint m_size;
        uint m_filler;
        
        
    public:
        vector_slice() : ref( nullptr ), m_start( 0 ), m_size( 0 ), m_filler( 0 ) {}
        vector_slice(std::vector<T> &reference ) {
            init( reference );
        }
        void init(std::vector<T> &reference ) {
            drop();
            if ( !reference.empty() ) {
                ref = reference.data();
                m_size = reference.size();
            }
        }
        vector_slice( vector_slice<T> &reference, uint start, uint size ) {
            init( reference, start, size );
        }
        void init( vector_slice<T> &reference, uint start, uint size ) {
            throw_assert( start + size <= reference.size(), "Slice outside reference." );
            drop();
            ref = reference.data() + start;
            m_size = size;
            m_start = start + reference.m_start;
        }
        vector_slice(std::vector<T> &reference, uint start, uint size ) {
            init( reference, start, size );
        }
        void init(std::vector<T> &reference, uint start, uint size ) {
            throw_assert( start + size <= reference.size(), "Slice outside reference." );
            drop();
            if ( !reference.empty() ) {
                m_start = start;
                ref = reference.data() + start;
                m_size = size;
            }
        }
        void init( T *reference, uint start, uint size ) {
            drop();
            m_start = start;
            ref = reference + start;
            m_size = size;
            
        }
        void drop() {
            ref = nullptr;
            m_start = 0;
            m_size = 0;
            m_filler = 0;
        }
        uint size() const {
            return m_size;
        }
        
        T &operator[]( uint i ) {
            throw_assert( i < m_size, "ArraySlice[] OOB" );
            return ref[i];
        }
        const T &operator[]( uint i ) const {
            throw_assert( i < m_size, "ArraySlice[] OOB" );
            return ref[i];
        }
        void set_zero() {
            for ( uint i : urange( m_size ) )
                ref[i] = 0;
        }
        T *begin() {
            return ref;
        }
        T *end() {
            return ref + m_size;
        }
        const T *begin() const {
            return ref;
        }
        const T *end() const {
            return ref + m_size;
        }
        
        void retarget(std::vector<T> &new_target ) {
            throw_assert( m_start + m_size <= new_target.size(), "Slice retarget outside reference." );
            ref = new_target.data() + m_start;
        }
        
        void swap( uint a, uint b ) {
            throw_assert( a < m_size && b < m_size, "ArraySlice swap() OOB." );
            if ( a == b )
                return;
            T temp = ref[a];
            ref[a] = ref[b];
            ref[b] = temp;
        }
        
        //No check !!!
        void resize( uint new_size ) {
            m_size = new_size;
        }
};



/*
    Optimization of the Array type for booleans: they are stored bitwise in 64 bit integers
*/
class vector_bool {
        class BoolMirror {
            private:
                uchar   pos;
                ulong  &container;
            public:
                BoolMirror( uchar pos, ulong &container ) : pos( pos ), container( container ) {}
                bool            value() const { return isBitHigh( container, pos ); }
                operator bool() const { return value(); }
                bool            operator= ( const bool &val ) {
                    if ( val ) {
                        setBitHigh( container, pos );
                        return true;
                    }
                    else {
                        setBitLow( container, pos );
                        return false;
                    }
                }
                bool            operator= ( const BoolMirror &mirror ) {
                    return this->operator=( mirror.value() );
                }
        };
        
        class Iterator {
			vector_bool&array_ref;
                uint pos;
            public:
                Iterator(vector_bool&array_ref, uint pos ) : array_ref( array_ref ), pos( pos ) {}
                auto operator*() { return array_ref[pos]; }
                auto operator++() {
                    pos++;
                    return *this;
                }
                bool operator !=( const Iterator &other ) {
                    return other.pos != pos;
                }
        };
        
        
        
        static uint get_array_size( uint size ) {
            return ( size - 1 ) / sizeof( ulong ) + 1;
        }
        static inline BoolMirror get_bool_mirror( std::vector<ulong> &data, uint pos ) {
            return BoolMirror( pos % sizeof( ulong ), data[pos / sizeof( ulong )] );
        }
        static inline bool get_bool( const std::vector<ulong> &data, uint pos ) {
            return isBitHigh( data[pos / sizeof( ulong )], pos % sizeof( ulong ) );
        }
        
        
		std::vector<ulong> data;
        uint m_size;
        
    public:
		vector_bool() : m_size( 0 ) {}
        
        //Creates an owner array of size <size>
        //A bool array is initialized to false everywhere
		vector_bool( uint size ) {
            init( size );
        }
        
        void init( uint size ) {
            m_size = size;
            if ( size != 0 ) {
                uint array_size = get_array_size( size );
                data.resize( array_size );
                set_zero();
            }
            else
                data.clear();
        }
        
        void drop() {
            data.clear();
            m_size = 0;
        }
        
        BoolMirror operator[]( uint i ) {
            throw_assert( i < m_size, "Array<bool> OOB" );
            return get_bool_mirror( data, i );
        }
        const bool operator[]( uint i ) const {
            throw_assert( i < m_size, "Array<bool> OOB" );
            return get_bool( data, i );
        }
        void set_zero() {
			std::fill(data.begin(), data.end(), 0);
        }
        
        auto begin() {
            return Iterator( *this, 0 );
        }
        auto end() {
            return Iterator( *this, m_size );
        }
        uint size() const {
            return m_size;
        }
        void swap( uint a, uint b ) {
            if ( a == b )
                return;
                
            bool temp = ( *this )[a];
            ( *this )[a] = ( *this )[b];
            ( *this )[b] = temp;
        }
        
        
        void resize( uint new_size );
};


/*
    The functions and objects in this namespace allow the color formatting of the console output.

    use ConsoleColor::Console::init() and ConsoleColor::Console::drop() at the start and end of the program
    to ensure the coloring works under Windows.

    ConsoleColor::Console::set_color(Color) will then change the color of the output for the next writes to the console.
*/
#if defined _WIN32 || defined _WIN64
struct ConsoleColor {
    enum ColorValue {
        BLACK = 0,
        DARK_BLUE = 1,
        DARK_GREEN = 2,
        TURQUOISE = 3,
        DARK_RED = 4,
        PURPLE = 5,
        DARK_YELLOW = 6,
        LIGHT_GRAY = 7,
        DARK_GRAY = 8,
        BLUE = 9,
        GREEN = 10,
        LIGHT_BLUE = 11,
        RED = 12,
        PINK = 13,
        YELLOW = 14,
        WHITE = 15,
        DEFAULT = 7
    };
    ColorValue text_color;
    ColorValue bg_color;
    ConsoleColor( ColorValue text_color = DEFAULT, ColorValue bg_color = BLACK ) : text_color( text_color ), bg_color( bg_color ) {}
    int get() {
        return text_color + ( bg_color * 16 );
    }
    bool operator!=( const ConsoleColor &other ) {
        return text_color != other.text_color || bg_color != other.bg_color;
    }
};


#else
struct ConsoleColor {
    enum ColorValue {
        BLACK		= 30,
        DARK_RED	= 31,
        DARK_GREEN	= 32,
        DARK_YELLOW = 33,
        DARK_BLUE	= 34,
        PURPLE		= 35,
        TURQUOISE	= 36,
        LIGHT_GRAY	= 37,
        DARK_GRAY	= 90,
        RED			= 91,
        GREEN		= 92,
        YELLOW		= 93,
        BLUE		= 94,
        PINK		= 95,
        LIGHT_BLUE	= 96,
        WHITE		= 97,
        DEFAULT		= 37
    };
    ColorValue text_color;
    ColorValue bg_color;
    ConsoleColor( ColorValue text_color = DEFAULT, ColorValue bg_color = BLACK ) : text_color( text_color ), bg_color( bg_color ) {}
    bool operator!=( const ConsoleColor &other ) {
        return text_color != other.text_color || bg_color != other.bg_color;
    }
};
#endif

struct Console {
    ConsoleColor current_color;
    

    public:
        Console();
        ~Console();
        
        void set_color( ConsoleColor which );
        inline void set_color( ConsoleColor::ColorValue which ){
			set_color( ConsoleColor( which ) );
		}
        
        void test_color();
        
        
#if defined _WIN32 || defined _WIN64
    private:
        void *hstdout;
        unsigned char csbi[22];
#endif

};

extern Console uconsole;

namespace STR {
    /*
        Splits the given string accoring to one or two given CHAR delimiters.
    */
    std::vector<std::string> split(const std::string& input, const char& delim);
    std::vector<std::string> split(const std::string& input, const char& delim, const char& delim2);

    /*
        Returns true if c is between 'A' and 'Z' or 'a' and 'z'
    */
    bool is_basic_letter(const char& c);
}


/*
    The FileReader struct provides helper functions to open a file, verify that it was correctly opened and dump its content into an
    Array object.
    The file is closed once the content has been read to the array.
    The read() function initiates the array with the size of the file and fills it with its content.
*/
struct FileReader {
    std::ifstream       file;
    bool open(const fs::path& file_path) {
        file = std::ifstream(file_path, std::ios::in | std::ios::binary | std::ios::ate );
        return file.is_open();
    }
    
    void read( std::vector<char> &target ) {
        auto pos = file.tellg();
        uint size = ( uint )pos;
        target.resize( size + 1LL );
        file.seekg( 0, std::ios::beg );
        file.read( target.data(), size );
        file.close();
        target[size] = '\0';
    }
    
    void read( std::vector<uchar> &target ) {
        auto pos = file.tellg();
        uint size = ( uint )pos;
        target.resize( size + 1LL );
        file.seekg( 0, std::ios::beg );
        file.read( ( char * )target.data(), size );
        file.close();
        target[size] = '\0';
    }
};

/*
    This function undecorates the C++ style name decoration of library functions.
    Only has an effect on Windows.
*/
bool undercorate_function_name( const std::string &name, std::vector<char> &buffer );

#include <iomanip>

/*
    Returns a string containing the formatted hexadecimal representation of the input.
*/
inline std::string to_hex( ulong val, uchar size = 16, bool prefix = false ) {
    char buff[24];
    if ( size == 0 ) {
        if ( prefix )
            sprintf( buff, "%#" PRIX64, val );
        else
            sprintf( buff, "%" PRIX64,  val );
    }
    else {
        if ( size > 16 )
            size = 16;
        if ( prefix )
            sprintf( buff, "%#0*" PRIX64, size, val );
        else
            sprintf( buff, "%0*" PRIX64, size, val );
    }
    return buff;
}






/*
    The log struct can be used to print preformatted output to the console.
    The different LogStreams available have different prefixes and colors.
    The prefix has to be explicitely included using Log::tag
    Use "\n" for new lines

    Ex.
    Log::err << Log::tag << "An error occured\n";
*/
namespace Log {


    struct OStreamTarget {
        virtual void print(const char* str, ConsoleColor color, const char* name) = 0;

        virtual ~OStreamTarget() {}
    };

    extern std::unique_ptr<OStreamTarget> output_stream;

    struct STDOutput : public OStreamTarget {

        void print(const char* str, ConsoleColor color, const char* name) {
            uconsole.set_color( color );
            printf("%s\n", str);
        }
    };
    constexpr int LARGE_BUFFER_SIZE = 4096;
    extern char LARGE_BUFFER[LARGE_BUFFER_SIZE];

    struct TagStruct {};
    extern TagStruct tag;
    struct PostStruct {};
    extern PostStruct post;
    class LogStream {
            std::stringstream ss;
        public:
            ConsoleColor const color;
            const char* const tag;
            const char* const name;
            bool hide;
            LogStream( ConsoleColor color, const char *tag, const char *name ) : color( color ), tag( tag ), name(name), hide( false ) {}

            void log(const char *format, ...) {
                va_list args;
                va_start(args, format);
                vsnprintf(LARGE_BUFFER, LARGE_BUFFER_SIZE, format, args);
                va_end(args);
                output_stream->print(LARGE_BUFFER, color, name);
            }
            void log_tag(const char* format, ...) {
                auto count = snprintf(LARGE_BUFFER, LARGE_BUFFER_SIZE, "%-7s", tag);
                va_list args;
                va_start(args, format);
                vsnprintf(LARGE_BUFFER+ count, LARGE_BUFFER_SIZE- count, format, args);
                va_end(args);
                output_stream->print(LARGE_BUFFER, color, name);
            }
    };
    
    extern LogStream info;
    extern LogStream err;
    extern LogStream debug;
    extern LogStream note;
    extern LogStream assert;
    extern LogStream test;
    extern LogStream ap; // For output from the autopilot

        
    extern LogStream sys;
    extern LogStream mem_read;
    extern LogStream mem_write;
    extern LogStream mem_fetch;
    extern LogStream code;
    extern LogStream instruction_operands;
    extern LogStream cache_hit_ratio;
    extern LogStream mem_access;
    extern LogStream reg;
    extern LogStream new_val;
    extern LogStream white;
}

/*
    Allows to progressively add values together and get the mean average of
    all of them.
    The mean_avg() can be called anytime and will give the average of the values that were
    registered until then with add().
*/
struct MeanAvgCollector {
    ulong count;
    ulong sum;
    MeanAvgCollector() : count( 0 ), sum( 0 ) {}
    ulong mean_avg() {
        if ( count > 0 )
            return sum / count;
        else
            return 0;
    }
    void add( ulong value ) {
        sum += value;
        ++count;
    }
    void reset() {
        count = 0;
        sum = 0;
    }
};


/*
    Uses a precice timer (cpu_clock based) to compute runtime deltas by using
    start(), end() then getDelta()
*/
struct PrecisionTimer {
    using HRClock = std::chrono::high_resolution_clock;
    using TimePoint = HRClock::time_point;
    static inline ulong time_delta_microsec(TimePoint start, TimePoint end) {
        return std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
    }
    static std::string microsecondTimeToString(ulong time) {
        if (time >= 1000000)
            return std::to_string(time / 1000000) + "." + std::to_string((time / 1000) % 1000) + "s";
        else if (time >= 1000)
            return std::to_string(time / 1000) + "." + std::to_string(time % 1000) + "ms";
        else
            return std::to_string(time) + "us";
    }
protected:
    TimePoint m_start, m_end;
public:
    std::string name;
    inline void start() {
        m_start = HRClock::now();
    }
    inline void end() {
        m_end = HRClock::now();
    }
    //Returns timer delta in microseconds
    inline long getDelta() {
        return (long)time_delta_microsec(m_start, m_end);
    }
    std::string print(std::string tab = std::string()) {
        return microsecondTimeToString(getDelta());
    }
};




/*
    Multi-platform wrapping of a dynamically loaded native library.
    init() will attempt to load a dynamic library from its name.
    The OS tries to locate the library first within the working directory of the program
    then in the system PATH.
    A full path and name can also be used.

    get_function() returns a pointer to the named function of the library or
    nullptr on error.
*/
struct Library {
    enum class OsType {
        WINDOWS,
        LINUX
    };
    #if defined _WIN32 || defined _WIN64
    static constexpr OsType type = OsType::WINDOWS;
    #else
    static constexpr OsType type = OsType::LINUX;
    #endif
    void *handle;
    Library() : handle( nullptr ) {}
    void init( const fs::path &file ); //No extension
    bool loaded() {
        return handle != nullptr;
    }
    
    void *get_function( const char *name, bool optional = false );
    
    ~Library();
};

template<typename T>
inline T abs_t( const T &val ) {
    if ( val >= 0 )
        return val;
    return -val;
}




bool json_get(const json& j, const char* entry, bool& target);
bool json_get(const json& j, const char* entry, int& target);
bool json_get(const json& j, const char* entry, uint& target);
bool json_get(const json& j, const char* entry, ulong& target);
bool json_get(const json& j, const char* entry, std::string& target);
