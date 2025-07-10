/**
 * (c) https://github.com/MontiCore/monticore
 *
 * The license generally applicable for this project
 * can be found under https://github.com/MontiCore/monticore.
 */
/* (c) https://github.com/MontiCore/monticore */
package de.monticore.lang.monticar.cnntrain._symboltable;

public enum RLAlgorithm {
    DQN {
        @Override
        public String toString() {
            return "dqn";
        }
    },
    DDPG {
        @Override
        public String toString() {
            return "ddpg";
        }
    },
    TD3 {
        @Override
        public String toString() {
            return "td3";
        }
    }
}