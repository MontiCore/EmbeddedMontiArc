/**
 * (c) https://github.com/MontiCore/monticore
 */
#include "config.h"

using namespace std;


bool str_equal( const char *first, uint size, const char *second ) {
    for ( uint i : urange( size ) )
        if ( second[i] == '\0' || second[i] != first[i] )
            return false;
    return second[size] == '\0';
}
