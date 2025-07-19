/**
 * (c) https://github.com/MontiCore/monticore
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
