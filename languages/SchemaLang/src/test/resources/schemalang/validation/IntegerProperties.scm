/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema IntegerProperties {

    integer_property1: Z
    integer_property2: Z
    integer_property3: Z
    integer_list_property: Z*
    integer_with_range_property_min: Z[-5:5]
    integer_with_range_property_max: Z[-5:5]
    integer_with_range_property_between: Z(-5:5)
    custom_property: myCustomProperty
    myCustomProperty {
        values:
            defined_instance;

        custom_integer_property: Z

        define defined_instance {
            defined_instance_integer_property: Z
        }
    }
}