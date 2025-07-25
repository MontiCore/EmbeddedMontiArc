/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema StringProperties {

    string_property: string
    string_list_property: string*

    custom_property: myCustomProperty

    myCustomProperty {
        values:
            defined_instance;

        custom_string_property: string

        define defined_instance {
            defined_instance_string_property: string
        }
    }
}