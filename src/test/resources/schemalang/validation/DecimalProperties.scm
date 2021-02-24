/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema DecimalProperties {

    decimal_property1: Q
    decimal_property2: Q
    decimal_property3: Q
    decimal_property4: Q
    decimal_list_property: Q*
    decimal_with_range_property_min: Q[-1:1]
    decimal_with_range_property_max: Q[-1:1]
    decimal_with_range_property_between: Q[-1:1]
    decimal_with_range_and_scale_property: Q[-1:0.1:1]

    custom_property: myCustomProperty

    myCustomProperty {
        values:
            defined_instance;

        custom_decimal_property: Q

        define defined_instance {
            defined_instance_decimal_property: Q
        }
    }
}