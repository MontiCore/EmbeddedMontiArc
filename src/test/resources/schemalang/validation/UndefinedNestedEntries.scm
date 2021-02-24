/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema UndefinedNestedEntries {

    some_configuration: some_object

    some_object {
        values:
            some_value;

        is_defined: Z
    }
}