/**
 * (c) https://github.com/MontiCore/monticore
 */
#pragma once


template<typename T>
inline T combine( T val ) {
    T res = 4;
    res += val;
    res /= 5;
    return res;
}

template<typename T, typename... Args>
inline T combine( T val, Args... args ) {
    T res = 3;
    res += val;
    res += combine<T>( args... );
    res /= 2;
    return res;
}


template<typename T>
inline T combine_array( T *val, int size ) {
    T res = 4;
    for ( int i = 0; i < size; ++i )
        res += val[i];
    res /= 5;
    return res;
}
