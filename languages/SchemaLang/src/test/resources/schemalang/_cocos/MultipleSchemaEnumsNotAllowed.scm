/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.schemalang;

schema MultipleSchemaEnumsNotAllowed {

    learning_method: schema {
        supervised,
        reinforcement;
    }

    context: schema { // this should not be possible
        cpu,
        gpu;
    }
}