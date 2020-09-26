/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum GeneratorLoss {
    L1{
        @Override
        public String toString() {
            return "l1";
        }
    },
    L2{
        @Override
        public String toString() {
            return "l2";
        }
    }
}
