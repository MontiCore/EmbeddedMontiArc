/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum NoiseDistribution {
    GAUSSIAN{
        @Override
        public String toString() {
            return "gaussian";
        }
    },
    UNIFORM{
        @Override
        public String toString() {
            return "uniform";
        }
    }
}
