/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema InitialValueCompatibleWithType {

    /* Initial value is a vector */
    list_of_strings = ("first", "second", "third"): string*
    list_of_integers = (10000, 0, -11111): Z*
    list_of_rationals = (0.33333333, 0.000001, 500): Q*
    list_of_booleans = (true, false, true): B*
    list_of_naturals = (1, 2, 3): N1*

    /* Component type */
    my_component = de.rwth.emadl.MyComponent: component

    /* Initial value is null */
    null_attribute = null: string

    /* EMA Types */
    natural_number_with_zero = 0: N
    natural_number_without_zero = 1: N1
    integer = -1: Z
    rational = 0.25: Q
    yes_no = true: B
    name = "hello world": string
}