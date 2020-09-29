#pragma once

#include <cstdint>

using i8 = int8_t;
using i16 = int16_t;
using i32 = int32_t;
using i64 = int64_t;

using u8 = uint8_t;
using u16 = uint16_t;
using u32 = uint32_t;
using u64 = uint64_t;

using f32 = float;
using f64 = double;

template<typename T>
struct vec2 {
    T x;
    T y;
    vec2<T>( T x, T y ) : x( x ), y( y ) {}
    vec2<T>() {}
    vec2<T>( T val ) : x( val ), y( val ) {}
    
    
    vec2<T> operator+( const vec2<T> &add ) const {
        return vec2<T>( x + add.x, y + add.y );
    }
    vec2<T> operator-( const vec2<T> &sub ) const {
        return vec2<T>( x - sub.x, y - sub.y );
    }
    
    //Component-wise multiplication
    inline vec2<T> operator*( const vec2<T> &mult ) const {
		return vec2<T>( x * mult.x, y * mult.y );
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
    
    //Cartesian length of the vector
    T operator!() const {
        return ( T ) sqrt( x * x + y * y );
    }
    
    //Cartesian length of the vector, also use !vec
    T length() const {
        return !*this;
    }
    
    //Returns normalized vector
    vec2 operator~() const {
        auto l = length();
        return l == 0 ? *this : *this / l;
    }
    
    //Returns a the normalized vector, also use ~vec.
    vec2 normalized() const {
        return ~*this;
    }
    
    template<typename A>
    explicit operator vec2<A>() const {
        return vec2<A>( ( A )x, ( A )y );
    }
};