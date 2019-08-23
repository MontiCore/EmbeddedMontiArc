/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum Context {
    CPU{
        @Override
        public String toString() {
            return "cpu";
        }
    },
    GPU{
        @Override
        public String toString() {
            return "gpu";
        }
    }
}
