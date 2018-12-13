#pragma once
#include <inttypes.h>
#include <exception>
#include <string>
#include <sstream>
#include <iostream>
#include <fstream>

namespace Utility {

    void write_uint64_t( char *mem_pos, uint64_t value );
    uint64_t read_uint64_t( char *mem_pos );
    
    
    void color_def();
    void color_err();
    void color_sys();
    void color_mem_read();
    void color_mem_write();
    void color_mem_fetch();
    void color_code();
    void color_reg();
    void color_new();
}



#include <cstdint>

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





/*


RANGE


*/
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

template<typename T>
class Range {
    public:
    
        Range( T start, T end ) : m_start( start ), m_end( end ) {}
        Range() : m_start( 0 ), m_end( 0 ) {}
        Range( T size ) : m_start( 0 ), m_end( size ) {}
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
//
template<typename T, typename V> Range( T, V )->Range<V>;








//Type definition for a unique_ptr referencing an array.
template <typename T>
using UniqueArray = std::unique_ptr<T[], std::default_delete<T[]>>;





/*
A Array is a static sized array for a given template type <T>, with the operator[] overloaded.
Use setZero() to set all bytes (independent of type) to 0.
Use begin() and end() return the iterator boundaries.
*/
template<typename T>
class Array {
        UniqueArray<T> data;
        uint m_size;
        
        
    public:
        Array() : m_size( 0 ) {}
        
        Array( uint size ) {
            init( size );
        }
        void init( uint size ) {
            drop();
            m_size = size;
            if ( size > 0 )
                data = UniqueArray<T>( new T[size] );
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
            
            for ( uint i : Range( min_size ) )
                new_data[i] = std::move( data[i] );
                
            data.reset();
            data = std::move( new_data );
            m_size = new_size;
        }
        
        void copy( Array &other ) {
            if ( m_size != other.size() )
                init( other.size() );
            for ( uint i : Range( m_size ) )
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
        void drop() {
            ref = nullptr;
            m_start = 0;
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
            for ( uint i : Range( m_size ) )
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
        static inline typename BoolMirror get_bool_mirror( Array<ulong> &data, uint pos ) {
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
        
        typename BoolMirror operator[]( uint i ) {
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


template<class T>
class IndexArray {
    public:
        class Iterator {
                IndexArray &index_array;
                uint pos;
            public:
                Iterator( IndexArray &index_array, uint pos ) : index_array( index_array ), pos( pos ) {}
                
                auto &operator*() {
                    return index_array[pos];
                }
                Iterator operator++() {
                    pos++;
                    return *this;
                }
                bool operator !=( const Iterator &other ) {
                    return other.pos != pos;
                }
        };
        
        
        
        IndexArray( Array<T> &target ) : target( &target ), m_size( target.size() ),
            m_indices( target.size() ) {}
        IndexArray() : target( nullptr ), m_size( 0 ) {}
        
        void init( Array<T> &target ) {
            drop();
            this->target = &target;
            this->m_size = target.size();
            this->m_indices.init( target.size() );
        }
        
        IndexArray &operator=( IndexArray & ) = delete;
        IndexArray( IndexArray & ) = delete;
        
        
        void fill_oneToOne() {
            throw_assert( target != nullptr, "Empty IndexArray" );
            
            for ( uint i : Range( m_size ) )
                setIndex( i, i );
        }
        
        void drop() {
            m_indices.drop();
            m_size = 0;
            target = nullptr;
        }
        
        uint size() {
            return m_size;
        }
        void setIndex( uint pos, uint target ) {
            throw_assert( pos < m_indices.size(), "OOB IndexArray" );
            m_indices[pos] = target;
        }
        uint getIndex( uint pos ) {
            throw_assert( pos < m_indices.size(), "OOB IndexArray" );
            return m_indices[pos];
        }
        auto &operator[]( uint pos ) {
            throw_assert( target != nullptr, "Empty IndexArray" );
            throw_assert( pos < m_size, "OOB IndexArray" );
            return ( *target )[m_indices[pos]];
        }
        
        auto &indices() {
            return m_indices;
        }
        auto &getTarget() {
            throw_assert( target != nullptr, "Empty IndexArray" );
            return *target;
        }
        
        typename Iterator begin() {
            return Iterator( *this, 0 );
        }
        typename Iterator end() {
            return Iterator( *this, m_size );
        }
        
        operator bool() {
            return target != nullptr;
        }
        
        //Manually sets a size for the target array, carefull use
        void updateSize( uint size ) {
            this->m_size = size;
        }
    private:
    
        Array<T> *target;
        uint m_size;
        Array<uint> m_indices;
        
};

template< typename T >
class Stack {
        T *data;
        uint m_count;
        uint m_size;
    public:
        Stack() : m_count( 0 ), data( nullptr ), m_size( 0 ) {}
        Stack( Array<T> &data ) {
            init( data );
        }
        void init( Array<T> &data ) {
            clear();
            this->data = data.begin();
            this->m_size = data.size();
        }
        Stack( ArraySlice<T> &data ) {
            init( data );
        }
        void init( ArraySlice<T> &data ) {
            clear();
            this->data = data.begin();
            this->m_size = data.size();
        }
        void clear() {
            m_count = 0;
        }
        bool more() const {
            return m_count > 0;
        }
        uint count() const {
            return m_count;
        }
        T pop() {
            throw_assert( m_count > 0, "Pop on empty Stack" );
            --m_count;
            return data[m_count];
        }
        void push( T v ) {
            throw_assert( m_count < m_size, "Push on full Stack" );
            data[m_count] = v;
            ++m_count;
        }
        
        T *get() const {
            return data;
        }
};

/*
Compact implementation of a static size rotation buffer queue.
Use more() to check if the queue has elements left.
Use pop() to get the next (=first) element of the queue. Removes the element from the queue.
Use push() to add an element at the end of the queue.
*/
template< typename T >
class Queue {
        T *data;
        uint m_start, m_end;
        uint m_count;
        uint m_size;
    public:
        Queue() : m_count( 0 ), data( nullptr ), m_size( 0 ), m_start( 0 ), m_end( 0 ) {}
        Queue( Array<T> &data ) {
            init( data );
        }
        void init( Array<T> &data ) {
            clear();
            this->data = data.begin();
            this->m_size = data.size();
        }
        Queue( ArraySlice<T> &data ) {
            init( data );
        }
        void init( ArraySlice<T> &data ) {
            clear();
            this->data = data.begin();
            this->m_size = data.size();
        }
        void clear() {
            m_count = 0;
            m_start = 0;
            m_end = 0;
        }
        bool more() const {
            return m_count > 0;
        }
        uint count() const {
            return m_count;
        }
        T pop() {
            throw_assert( m_count > 0, "Pop on empty Queue" );
            uint p = m_end;
            m_end = ( m_end + 1 ) % m_size;
            --m_count;
            return data[p];
        }
        void push( T v ) {
            throw_assert( m_count < m_size, "Push on full Queue" );
            if ( m_count < m_size ) {
                data[m_start] = v;
                ++m_count;
                m_start = ( m_start + 1 ) % m_size;
            }
        }
        
        T *get() const {
            return data;
        }
};




namespace ConsoleColor {
    #ifdef WIN32
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
    
#define B_WHITE "\033[38;5;16;48;5;231m"
#define B_RED "\033[38;5;16;48;5;196m"
#define B_GREEN "\033[38;5;16;48;5;46m"
#define B_LIGHT_BLUE "\033[38;5;16;48;5;51m"
#define B_YELLOW "\033[38;5;16;48;5;220m"
#define B_DARK_BLUE "\033[38;5;250;48;5;21m"
#define B_BLUE "\033[38;5;231;48;5;33m"
    
#define RED "\033[38;5;196m"
#define GREEN "\033[38;5;46m"
#define BLUE "\033[38;5;33m"
#define LIGHT_BLUE "\033[38;5;51m"
#define YELLOW "\033[38;5;220m"
    
#define W_RED "\033[38;5;231;48;5;196m"
#define W_GREEN "\033[38;5;224;48;5;46m"
    
#define DEFAULT "\033[0m"
    
    struct Color {
    
        public:
            static void init();
    
            static void close();
            static void c( const char *which );
    
            static void testColor();
    };
    
    #endif
    
    
}

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
};


bool undercorate_function_name( const std::string &name, Array<char> &buffer );