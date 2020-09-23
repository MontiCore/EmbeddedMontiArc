/**
 *
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnnarch._symboltable;

public enum ParameterType {
    LAYER_PARAMETER {
        @Override
        public String toString(){
            return "layer parameter";
        }
    },
    ARCHITECTURE_PARAMETER {
        @Override
        public String toString(){
            return "architecture parameter";
        }
    },
    CONSTANT {
        @Override
        public String toString(){
            return "constant";
        }
    },
    TIME_PARAMETER {
        @Override
        public String toString(){
            return "time parameter";
        }
    },
    UNKNOWN {
        //describes a parameter which does not exist. Only used to avoid exceptions while checking Cocos.
        @Override
        public String toString(){
            return "unknown";
        }
    }
}
