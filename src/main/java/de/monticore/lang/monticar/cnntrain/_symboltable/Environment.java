/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

/**
 *
 */
public enum Environment {
    ROS_INTERFACE {
        @Override
        public String toString() {
            return "ros-interface";
        }
    },
    GYM {
        @Override
        public String toString() {
            return "gym";
        }
    }
}
