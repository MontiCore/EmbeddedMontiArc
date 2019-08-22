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
#pragma once
#include <inttypes.h>
#include <exception>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <memory>
#include <cstring>
#include <cmath>
#include <list>

namespace Utility {

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

extern uint BIT_MASKS[];



/*
    DEBUG MODE ASSERTIONS
*/

struct AssertionFailureException : public std::exception {
    AssertionFailureException( const char *expression, const char *file, int line,
                               const char *message ) {
        std::ostringstream outputStream;
        
        std::string message_string( message );
        if ( !message_string.empty() )
            outputStream << message_string << ": ";
            
        std::string expressionString = expression;
        if ( expressionString == "false" || expressionString == "0" || expressionString == "FALSE" )
            outputStream << "Unreachable code assertion";
        else
            outputStream << "Assertion '" << expression << "'";
            
        outputStream << " failed in file '" << file << "' line " << line;
        std::cerr << outputStream.str() << std::endl;
    }
};

// Assert that EXPRESSION evaluates to true, otherwise raise AssertionFailureException with associated MESSAGE
#ifdef _DEBUG
    #define throw_assert(EXPRESSION, MESSAGE) if(!(EXPRESSION)) { throw AssertionFailureException(#EXPRESSION, __FILE__, __LINE__,  MESSAGE); }
#else
    #define throw_assert(E, M) ((void)0)
#endif






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








//Type definition for a unique_ptr referencing an array.
template <typename T>
using UniqueArray = std::unique_ptr<T[], std::default_delete<T[]>>;
/*
    An Array is a static sized array for a given template type <T>, with the operator[] overloaded.
    Use set_zero() to set all bytes (independent of type) to 0.
    Use begin() and end() return the iterator boundaries. This means the Array can be used in for loops:
    for(auto &e : array) {...}
    It is non copyable.
    It can be emptied with drop()
*/
template<typename T>
class Array {
        UniqueArray<T> data;
        uint m_size;
        
        
    public:
        Array() : m_size( 0 ) {}
        
        Array( uint size ) {
            m_size = 0;
            init( size );
        }
        void init( uint size ) {
            if ( size == m_size )
                return;
            drop();
            m_size = size;
            if ( size > 0 )
                data = UniqueArray<T>( new T[size] );
        }
        void init( uint size, T *buffer ) {
            init( size );
            std::memcpy( data.get(), buffer, sizeof( T )*m_size );
        }
        //*
        Array( const Array &x ) = delete;
        Array &operator=( const Array &x ) = delete;
        /*/
        Array( const Array &x ) : m_size( 0 ) {
        copy( x );
        }
        Array &operator=( const Array &x ) {
        copy( x );
        return *this;
        }
        //*/
        void drop() {
            data.reset();
            m_size = 0;
        }
        uint size() const {
            return m_size;
        }
        
        T &operator[]( uint i ) {
            throw_assert( i < m_size, "Array OOB" );
            return data[i];
        }
        const T &operator[]( uint i ) const {
            throw_assert( i < m_size, "Array OOB" );
            return data[i];
        }
        void set_zero() {
            if ( m_size > 0 )
                std::memset( data.get(), 0, sizeof( T )*m_size );
        }
        T *begin() {
            return data.get();
        }
        T *end() {
            return data.get() + m_size;
        }
        const T *begin() const {
            return data.get();
        }
        const T *end() const {
            return data.get() + m_size;
        }
        
        void swap( uint a, uint b ) {
            throw_assert( a < m_size && b < m_size, "Array swap() OOB." );
            if ( a == b )
                return;
            T temp = data[a];
            data[a] = data[b];
            data[b] = temp;
        }
        void resize( uint new_size ) {
            if ( new_size == m_size )
                return;
            if ( new_size == 0 ) {
                drop();
                return;
            }
            if ( m_size == 0 ) {
                this->init( new_size );
                return;
            }
            UniqueArray<T> new_data = UniqueArray<T>( new T[new_size] );
            uint min_size = new_size < m_size ? new_size : m_size;
            
            for ( uint i : urange( min_size ) )
                new_data[i] = std::move( data[i] );
                
            data.reset();
            data = std::move( new_data );
            m_size = new_size;
        }
        
        void copy( Array &other ) {
            if ( m_size != other.size() )
                init( other.size() );
            for ( uint i : urange( m_size ) )
                data[i] = other[i];
        }
        void mem_copy( Array &other ) {
            if ( other.m_size == 0 )
                return;
            m_size = other.m_size;
            auto p = new T[m_size];
            data = UniqueArray<T>( p );
            std::memcpy( p, other.data.get(), sizeof( T )*m_size );
        }
        
        void mem_copy( Array &other, uint count ) {
            throw_assert( count <= other.size(), "Copy outside source." );
            if ( count == 0 )
                return;
            m_size = count;
            auto p = new T[count];
            data = UniqueArray<T>( p );
            std::memcpy( p, other.data.get(), sizeof( T )*count );
        }
        
        bool empty() { return m_size == 0; }
};

/*
    An ArraySlice proxies the content or a part of the content of an other array or array slice.
    It overloads the operator[] and the iteration functions.
*/
template<typename T>
class ArraySlice {
        T *ref;
        uint m_start;
        uint m_size;
        uint m_filler;
        
        
    public:
        ArraySlice() : ref( nullptr ), m_start( 0 ), m_size( 0 ), m_filler( 0 ) {}
        ArraySlice( Array<T> &reference ) {
            init( reference );
        }
        void init( Array<T> &reference ) {
            drop();
            if ( !reference.empty() ) {
                ref = reference.begin();
                m_size = reference.size();
            }
        }
        ArraySlice( ArraySlice<T> &reference, uint start, uint size ) {
            init( reference, start, size );
        }
        void init( ArraySlice<T> &reference, uint start, uint size ) {
            throw_assert( start + size <= reference.size(), "Slice outside reference." );
            drop();
            ref = reference.begin() + start;
            m_size = size;
            m_start = start + reference.m_start;
        }
        ArraySlice( Array<T> &reference, uint start, uint size ) {
            init( reference, start, size );
        }
        void init( Array<T> &reference, uint start, uint size ) {
            throw_assert( start + size <= reference.size(), "Slice outside reference." );
            drop();
            if ( !reference.empty() ) {
                m_start = start;
                ref = reference.begin() + start;
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
        
        void retarget( Array<T> &new_target ) {
            throw_assert( m_start + m_size <= new_target.size(), "Slice retarget outside reference." );
            ref = new_target.begin() + m_start;
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
template<>
class Array < bool> {
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
                Array &array_ref;
                uint pos;
            public:
                Iterator( Array &array_ref, uint pos ) : array_ref( array_ref ), pos( pos ) {}
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
        static inline BoolMirror get_bool_mirror( Array<ulong> &data, uint pos ) {
            return BoolMirror( pos % sizeof( ulong ), data[pos / sizeof( ulong )] );
        }
        static inline bool get_bool( const Array<ulong> &data, uint pos ) {
            return isBitHigh( data[pos / sizeof( ulong )], pos % sizeof( ulong ) );
        }
        
        
        Array<ulong> data;
        uint m_size;
        
    public:
        Array() : m_size( 0 ) {}
        
        //Creates an owner array of size <size>
        //A bool array is initialized to false everywhere
        Array( uint size ) {
            init( size );
        }
        
        void init( uint size ) {
            m_size = size;
            if ( size != 0 ) {
                uint array_size = get_array_size( size );
                data.init( array_size );
                set_zero();
            }
            else
                data.drop();
        }
        
        void drop() {
            data.drop();
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
            data.set_zero();
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
namespace ConsoleColor {
    #if defined _WIN32 || defined _WIN64
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
    
    struct Color {
        ColorValue text_color;
        ColorValue bg_color;
        Color( ColorValue text_color = DEFAULT, ColorValue bg_color = BLACK ) : text_color( text_color ),
            bg_color( bg_color ) {}
        uint get() {
            return text_color + ( bg_color * 16 );
        }
        bool operator!=( const Color &other ) {
            return text_color != other.text_color || bg_color != other.bg_color;
        }
    };
    
    struct Console {
    
            static void *hstdout;
            static uchar csbi[22];
        public:
            static void init();
            
            static void drop();
            static void set_color( Color which );
            
            static void test_color();
    };
    
    
    #else
    
    extern const char *BLACK;
    extern const char *DARK_BLUE;
    extern const char *DARK_GREEN;
    extern const char *TURQUOISE;
    extern const char *DARK_RED;
    extern const char *PURPLE;
    extern const char *DARK_YELLOW;
    extern const char *LIGHT_GRAY;
    extern const char *DARK_GRAY;
    extern const char *BLUE;
    extern const char *GREEN;
    extern const char *LIGHT_BLUE;
    extern const char *RED;
    extern const char *PINK;
    extern const char *YELLOW;
    extern const char *WHITE;
    extern const char *DEFAULT;
    
    struct Color {
        const char *val;
        Color( const char *text_color = DEFAULT, const char *bg_color = BLACK ) : val( text_color ) {}
        const char *get() {
            return val;
        }
        bool operator!=( const Color &other ) {
            return val != other.val;
        }
    };
    
    struct Console {
    
        public:
            static void init() {}
    
            static void drop();
            static void set_color( Color which );
    
            static void test_color() {}
    };
    
    
    #endif
    
    
}

/*
    Replacement of some FileSystem functionnalities for compatibility.
*/
namespace FS {
    struct File {
        std::string path;
        std::string name;
        std::string extension;
        File(std::string folder, std::string name);
    };


    std::string append(const std::string &path, const std::string &file);
    std::string current_path();
    std::string canonical(const std::string &path);
    std::list<FS::File> directory_files(const std::string &folder );
    
}

/*
    The FileReader struct provides helper functions to open a file, verify that it was correctly opened and dump its content into an
    Array object.
    The file is closed once the content has been read to the array.
    The read() function initiates the array with the size of the file and fills it with its content.
*/
struct FileReader {
    std::ifstream       file;
    bool open( std::string name ) {
        file = std::ifstream( name, std::ios::in | std::ios::binary | std::ios::ate );
        return file.is_open();
    }
    
    void read( Array<char> &target ) {
        auto pos = file.tellg();
        uint size = ( uint )pos;
        target.init( size + 1 );
        file.seekg( 0, std::ios::beg );
        file.read( target.begin(), size );
        file.close();
        target[size] = '\0';
    }
    
    void read( Array<uchar> &target ) {
        auto pos = file.tellg();
        uint size = ( uint )pos;
        target.init( size + 1 );
        file.seekg( 0, std::ios::beg );
        file.read( ( char * )target.begin(), size );
        file.close();
        target[size] = '\0';
    }
};

/*
    This function undecorates the C++ style name decoration of library functions.
    Only has an effect on Windows.
*/
bool undercorate_function_name( const std::string &name, Array<char> &buffer );

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
struct Log {
        static ConsoleColor::Color current_color;
        struct TagStruct {};
        static TagStruct tag;
        class LogStream {
                ConsoleColor::Color color;
                const char *tag;
            public:
                bool hide;
                LogStream( ConsoleColor::Color color, const char *tag ) : color( color ), tag( tag ), hide( false ) {}
                LogStream &operator<<( const TagStruct &param ) {
                    if ( hide )
                        return *this;
                    if ( current_color != color ) {
                        current_color = color;
                        ConsoleColor::Console::set_color( color );
                    }
                    printf( "%-6s", tag );
                    return *this;
                }
                template<typename T>
                LogStream &operator<<( const T &param ) {
                    if ( hide )
                        return *this;
                    if ( current_color != color ) {
                        current_color = color;
                        ConsoleColor::Console::set_color( color );
                    }
                    std::cout << param;
                    return *this;
                }
        };
        
        static LogStream info;
        static LogStream err;
        static LogStream debug;
        static LogStream sys;
        static LogStream mem_read;
        static LogStream mem_write;
        static LogStream mem_fetch;
        static LogStream code;
        static LogStream reg;
        static LogStream new_val;
        static LogStream note;
        static LogStream white;
        static LogStream test;
        
};

/*
    Allows to progressively add values together and get the mean average of
    all of them.
    The mean_avg() can be called anytime and will give the average of the values that were
    registered with add().
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
    Multi-platform wrapping of a dynamically loaded library.
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
    bool init( const char *name );
    bool loaded() {
        return handle != nullptr;
    }
    
    void *get_function( const char *name );
    
    ~Library();
};

/*
Vector of length 2 and type <T>.
Members are accessed through x and y or the operator[].
Use length() to get the cartesian length of the vector.
A vector supports component-wise operations (+, -, *, /, %) for scalar values
and supports addition and substraction between vectors.
*/
template<typename T>
struct vec2 {
    union {
        struct {
            T x;
            T y;
        };
        T data[2];
        struct {
            T min;
            T max;
        };
    };
    vec2<T>( T x, T y ) : x( x ), y( y ) {}
    vec2<T>( T *data ) : x( data[0] ), y( data[1] ) {}
    vec2<T>() {}
    vec2<T>( T val ) : x( val ), y( val ) {}
    
    
    vec2<T> operator+( const vec2<T> &add ) const {
        return vec2<T>( x + add.x, y + add.y );
    }
    vec2<T> operator-( const vec2<T> &sub ) const {
        return vec2<T>( x - sub.x, y - sub.y );
    }
    
    vec2<T> &operator += ( const vec2<T> &add ) {
        x += add.x;
        y += add.y;
        return *this;
    }
    vec2<T> &operator -= ( const vec2<T> &sub ) {
        x -= sub.x;
        y -= sub.y;
        return *this;
    }
    
    vec2<T> operator-() const {
        return vec2<T>( -x, -y );
    }
    
#define vec_scalar_op(OP) vec2<T> operator OP ( const T& val ) const { return vec2<T>(x OP val, y OP val); }
    vec_scalar_op( * )
    vec_scalar_op( / )
    vec_scalar_op( + )
    vec_scalar_op( - )
    vec_scalar_op( % )
#undef vec_scalar_op
    
#define vec_scalar_op(OP) vec2<T> &operator OP ( const T& val ) { x OP val; y OP val; return *this; }
    vec_scalar_op( *= )
    vec_scalar_op( /= )
    vec_scalar_op( += )
    vec_scalar_op( -= )
    vec_scalar_op( %= )
#undef vec_scalar_op
    
    T length() const {
        return ( T )sqrt( x * x + y * y );
    }
    inline T &operator[]( uint i ) {
        throw_assert( i < 2, "Vector data access param too big" );
        return data[i];
    }
    inline const T &operator[]( uint i ) const {
        throw_assert( i < 2, "Vector data access param too big" );
        return data[i];
    }
    
    template<typename A>
    explicit operator vec2<A>() const {
        return vec2<A>( ( A )x, ( A )y );
    }
};



using  dvec2 = vec2<double>;

/*
    dot product of 2 vectors
*/
template<typename T>
inline T dot( const vec2<T> &v1, const vec2<T> &v2 ) {
    return v1.x * v2.x + v1.y * v2.y;
}

/*
    Returns the normalized <v> vector.
    Returns the zero vector if given the zero vector.
*/
template<template<typename> typename U, typename T>
inline U<T> normalize( const U<T> &v ) {
    T l = v.length();
    
    if ( l == 0 )
        return v;
        
    return v / l;
}

template<typename T>
inline T min_t( const T a, const T b ) {
    return a < b ? a : b;
}
template<typename T>
inline T max_t( const T a, const T b ) {
    return a > b ? a : b;
}
template<typename T>
inline void maximize( T &target, const T val ) {
    if ( val > target )
        target = val;
}
template<typename T>
inline void minimize( T &target, const T val ) {
    if ( val < target )
        target = val;
}

template<typename T>
inline T abs_t( const T &val ) {
    if ( val >= 0 )
        return val;
    return -val;
}

template<typename T>
inline void minimize_abs( T &target, const T val ) {
    if ( abs_t( val ) < abs_t( target ) )
        target = val;
}
