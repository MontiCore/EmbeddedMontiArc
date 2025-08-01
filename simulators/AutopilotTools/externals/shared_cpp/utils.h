/**
 * (c) https://github.com/MontiCore/monticore
 */
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

static constexpr float PIf = 3.14159265358979323846f;
static constexpr double PId = 3.14159265358979323846;

static constexpr float DEG_TO_RADf = 2.0f / 360.0f * PIf;
static constexpr double DEG_TO_RADd = 2.0 / 360.0 * PId;
static constexpr float RAD_TO_DEGf = 360.0f / ( 2.0f * PIf );
static constexpr double RAD_TO_DEGd = 360.0 / ( 2.0 * PId );

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
    T length() const {
        return ( T ) sqrt( x * x + y * y );
    }
    
    template<typename A>
    explicit operator vec2<A>() const {
        return vec2<A>( ( A )x, ( A )y );
    }

    T &operator[](size_t i) {
        if (i == 1) return y;
        return x;
    }
};

using vec2i8 = vec2<i8>;
using vec2i16 = vec2<i16>;
using vec2i32 = vec2<i32>;
using vec2i64 = vec2<i64>;

using vec2u8 = vec2<u8>;
using vec2u16 = vec2<u16>;
using vec2u32 = vec2<u32>;
using vec2u64 = vec2<u64>;

using vec2f32 = vec2<f32>;
using vec2f64 = vec2<f64>;

/*
Returns the normalized <v> vector.
Returns the zero vector if given the zero vector.
*/
template<template<typename> class U, typename T>
inline U<T> normalize( const U<T> &v ) {
    T l = v.length();
    if ( l == 0 )
        return v;
    return v / l;
}

template<template<typename> class U>
inline U<f64> normalize( const U<f64> &v ) {
    f64 l = v.length();
    if ( l < 0.00000001 && l > -0.00000001 )
        return U<f64>(0);
    return v / l;
}

template<template<typename> class U>
inline U<f32> normalize( const U<f32> &v ) {
    f32 l = v.length();
    if ( l < 0.000001f && l > -0.000001f )
        return U<f32>(0);
    return v / l;
}

template<typename T>
inline T dot( const vec2<T> &v1, const vec2<T> &v2 ) {
    return v1.x * v2.x + v1.y * v2.y;
}

template<typename T>
inline T distance( const vec2<T> &v1, const vec2<T> &v2 ) {
    return (v1-v2).length();
}