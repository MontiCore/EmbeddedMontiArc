/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum LRPolicy {
    FIXED{
        @Override
        public String toString() {
            return "fixed";
        }
    },
    STEP{
        @Override
        public String toString() {
            return "step";
        }
    },
    EXP{
        @Override
        public String toString() {
            return "exp";
        }
    },
    INV{
        @Override
        public String toString() {
            return "inv";
        }
    },
    POLY{
        @Override
        public String toString() {
            return "poly";
        }
    },
    SIGMOID{
        @Override
        public String toString() {
            return "sigmoid";
        }
    }
}
