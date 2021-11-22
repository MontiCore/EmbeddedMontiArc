/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema ConfigurationEntryWithIncompatibleValue {

    my_enum: enum {
        value1,
        value2;
    }
    boolean_property: B
    boolean_list: B*

    natural_numbers_property: N1
    natural_numbers_list: N1*
    natural_numbers_with_range: N1(10:20)
    natural_numbers_with_range_and_scale: N1(-10:5:10)

    integer_property: Z
    integer_list: Z*
    integer_with_range: Z(-10:10)
    integer_with_range_and_scale: Z(-10:2:10)

    rational_property: Q
    rational_list: Q*
    rational_with_range: Q(-1:1)
    rational_with_range_and_scale: Q(-1:0.1:1)

    string_property: string
    string_list: string*

    component_property: component
    custom_property: custom_property_type

    custom_property_type {
        values:
            defined_instance;
    }
}