/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema RequiredEntries {

    required: Q!
    required_enum: enum! {
        some_value;
    }
    required_nested: some_object!

    also_required: Q!
    also_required_enum: enum! {
        some_constant;
    }
    also_required_nested: some_object!

    some_object {
        values:
            some_value;
    }
}