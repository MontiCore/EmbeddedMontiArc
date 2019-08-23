/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

/**
 *
 */
public enum LearningMethod {
    SUPERVISED {
        @Override
        public String toString() {
            return "supervised";
        }
    },
    REINFORCEMENT {
        @Override
        public String toString() {
            return "reinforcement";
        }
    }
}
