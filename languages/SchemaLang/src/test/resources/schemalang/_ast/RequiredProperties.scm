/* (c) https://github.com/MontiCore/monticore */
// package de.serwth.emadl;

schema RequiredProperties {

    required_boolean: B!
    required_integer: Z!
    required_enum: enum! {
        hello_world;
    }
    required_complex: complex_object!

    not_required_boolean: B
    not_required_integer: Z
    not_required_enum: enum {
        hello_world;
    }
    not_required_complex: complex_object

    complex_object {
        values:
            some_value;
    }
}