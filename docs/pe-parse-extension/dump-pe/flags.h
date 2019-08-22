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
#include <parser-library/parse.h>

struct Flag {
    uint64_t bit;
    char *name;
};


extern Flag file_characteristics_flags[];
extern Flag machine_values[];
extern Flag signature_values[];
extern Flag subsystem_values[];
extern Flag dll_flags[];
extern Flag data_directory_values[];
extern Flag section_characteristics_flags[];
extern Flag section_align_values[];
extern uint64_t section_align_mask;

void print_flags_bit( Flag *flags, uint64_t value );
void print_flags_value( Flag *flags, uint64_t value );