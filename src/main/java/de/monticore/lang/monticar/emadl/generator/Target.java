package de.monticore.lang.monticar.emadl.generator;

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
            return ".cpp";
        }
    }
}
