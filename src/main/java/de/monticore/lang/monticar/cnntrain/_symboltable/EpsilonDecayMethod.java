/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

/**
 *
 */
public enum EpsilonDecayMethod {
    LINEAR {
        @Override
        public String toString() {
            return "linear";
        }
    },
    NO {
        @Override
        public String toString() {
            return "no";
        }
    }
}
