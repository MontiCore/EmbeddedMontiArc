/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema OnlyOneSchemaEnumDefined {

    learning_method: schema {
        supervised,
        reinforcement;
    }

    context: enum {
        cpu,
        gpu;
    }
}