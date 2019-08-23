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
#include "function_calling.h"





void StackCall::push_param_64( ulong p ) {
    stack.push_long( p );
}

void StackCall::push_param_32( uint p ) {
    stack.push_int( p );
}

ulong StackCall::pop_param_64() {
    return stack.pop_long();
}

uint StackCall::push_param_32() {
    return stack.pop_int();
}

ulong StackCall::get_return() {
    return registers.get_rax();
}

void StackCall::set_return( ulong r ) {
    registers.set_rax( r );
}
