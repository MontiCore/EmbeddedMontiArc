/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch.generator;

//can be removed
public enum Target {
    PYTHON{
        @Override
        public String toString() {
            return ".py";
        }
    },
    CPP{
        @Override
        public String toString() {
            return ".h";
        }
    }
}
