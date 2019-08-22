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
#include "base.h"
#include "algorithm.h"

EXPORT int32_t int_one( int32_t t1){
    return combine<int32_t>(t1);
}
EXPORT int32_t int_two( int32_t t1, int32_t t2 ){
    return combine<int32_t>(t1,t2);
}
EXPORT int32_t int_three( int32_t t1, int32_t t2, int32_t t3 ){
    return combine<int32_t>(t1,t2,t3);
}
EXPORT int32_t int_four( int32_t t1, int32_t t2, int32_t t3, int32_t t4){
    return combine<int32_t>(t1,t2,t3,t4);
}


EXPORT int64_t long_one( int64_t t1){
    return combine<int64_t>(t1);
}
EXPORT int64_t long_two( int64_t t1, int64_t t2 ){
    return combine<int64_t>(t1,t2);
}
EXPORT int64_t long_three( int64_t t1, int64_t t2, int64_t t3){
    return combine<int64_t>(t1,t2,t3);
}
EXPORT int64_t long_four( int64_t t1, int64_t t2, int64_t t3, int64_t t4){
    return combine<int64_t>(t1,t2,t3,t4);
}


EXPORT float float_one( float t1){
    return combine<float>(t1);
}
EXPORT float float_two( float t1,float t2 ){
    return combine<float>(t1,t2);
}
EXPORT float float_three( float t1,float t2,float t3){
    return combine<float>(t1,t2,t3);
}
EXPORT float float_four( float t1,float t2,float t3 ,float t4){
    return combine<float>(t1,t2,t3,t4);
}


EXPORT double double_one( double t1){
    return combine<double>(t1);
}
EXPORT double double_two( double t1,double t2 ){
    return combine<double>(t1,t2);
}
EXPORT double double_three( double t1,double t2,double t3){
    return combine<double>(t1,t2,t3);
}
EXPORT double double_four( double t1,double t2,double t3,double t4){
    return combine<double>(t1,t2,t3,t4);
}

EXPORT double double_array(double*a, int s){
    return combine_array<double>(a,s);
}
EXPORT float float_array(float*a, int s){
    return combine_array<float>(a,s);
}
EXPORT int32_t int_array(int32_t*a, int s){
    return combine_array<int32_t>(a,s);
}
EXPORT int64_t long_array(int64_t*a, int s){
    return combine_array<int64_t>(a,s);
}
EXPORT char char_array(char*a, int s){
    return combine_array<char>(a,s);
}