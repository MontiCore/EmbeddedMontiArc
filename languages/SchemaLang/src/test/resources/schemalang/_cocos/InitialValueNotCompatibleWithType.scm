/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema InitialValueNotCompatibleWithType {

    /* Initial value is a vector */
    list_of_strings = ("first", 0.5, "third"): string*
    list_of_integers = (0.0, 0, -11111): Z*
    list_of_rationals = ("my string", 0.000001, 500): Q*
    list_of_booleans = ("true", false, true): B*
    list_of_naturals = (-1, 2, 3): N*

    /* Component type */
    my_component = "de.rwth.emadl.MyComponent": component

    /* EMA Types */
    natural_number_with_zero = -1: N
    natural_number_without_zero = 0: N1
    integer = -0.25: Z
    rational = "0.25": Q
    yes_no = "true": B
    name = no_string: string
}